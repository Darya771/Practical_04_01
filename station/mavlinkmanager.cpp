#include "mavlinkmanager.h"

#include <QNetworkDatagram>
#include <limits>

using namespace Mavlink;

MavlinkManager::MavlinkManager(TelemetryModel &model, QObject *parent)
    : QObject(parent), m_model(model)
{
    m_linkTimer.setInterval(1000);
    m_watchdog.setSingleShot(true);
    m_watchdog.setTimerType(Qt::PreciseTimer);
    m_watchdog.setInterval(3000);
    m_ackTimer.setSingleShot(true);
    m_ackTimer.setTimerType(Qt::PreciseTimer);
    m_ackTimer.setInterval(3000);
    connect(&m_socket, &QUdpSocket::readyRead, this, &MavlinkManager::receive);
    connect(&m_linkTimer, &QTimer::timeout, this, &MavlinkManager::checkLink);
    connect(&m_watchdog, &QTimer::timeout, this, &MavlinkManager::checkLink);
    connect(&m_ackTimer, &QTimer::timeout, this, [this] {
        finishCommand(tr("Подтверждение не получено"));
    });
}

void MavlinkManager::open()
{
    if (m_link != Closed)
        return;
    if (!m_socket.bind(QHostAddress::LocalHost, StationPort, QUdpSocket::DontShareAddress)) {
        emit event(tr("Ошибка"), tr("Порт %1: %2").arg(StationPort).arg(m_socket.errorString()));
        return;
    }
    m_received = m_missing = m_invalid = 0;
    m_sender.clear();
    m_firstPacket = m_lastPacket = QDateTime();
    m_lastSeq = -1;
    m_link = Waiting;
    m_clock.start();
    m_linkTimer.start();
    emit event(tr("Связь"), tr("Открыт канал 127.0.0.1:%1").arg(StationPort));
    emit stateChanged();
}

void MavlinkManager::close()
{
    m_link = Closed;
    m_socket.close();
    m_linkTimer.stop();
    m_watchdog.stop();
    m_rate = 0;
    m_recent.clear();
    if (m_pending)
        finishCommand(tr("Ожидание прервано: канал закрыт"));
    emit event(tr("Связь"), tr("Канал закрыт"));
    emit stateChanged();
}

void MavlinkManager::receive()
{
    while (m_socket.hasPendingDatagrams()) {
        const auto datagram = m_socket.receiveDatagram();
        if (!datagram.isValid() || datagram.senderAddress() != QHostAddress::LocalHost
            || datagram.senderPort() != SimulatorPort)
            continue;

        Frame frame;
        QString error;
        if (!MavlinkMessageParser::decode(datagram.data(), frame, error)
            || frame.system != VehicleId || frame.component != ComponentId
            || frame.id == CommandLong) {
            ++m_invalid;
            emit event(tr("Ошибка пакета"), error.isEmpty() ? tr("Неожиданный источник или тип") : error);
            emit stateChanged();
            continue;
        }

        ++m_received;
        m_sender = QStringLiteral("%1:%2").arg(datagram.senderAddress().toString())
                                             .arg(datagram.senderPort());
        m_lastPacket = QDateTime::currentDateTime();
        if (!m_firstPacket.isValid())
            m_firstPacket = m_lastPacket;
        m_lastTick = m_clock.elapsed();
        m_watchdog.start();
        m_recent.enqueue(m_lastTick);

        if (m_link != Online) {
            const bool restored = m_link == Lost;
            m_link = Online;
            emit event(tr("Связь"), restored ? tr("Связь восстановлена") : tr("Связь установлена"));
        }

        checkSequence(frame.seq);
        applyFrame(frame);
        emit stateChanged();
    }
}

void MavlinkManager::checkSequence(quint8 seq)
{
    if (m_lastSeq >= 0) {
        const int delta = (256 + seq - m_lastSeq) % 256;
        if (delta == 0 || delta >= 128)
            return;
        if (delta > 1) {
            m_missing += delta - 1;
            emit event(tr("Пропуск"), tr("По полю seq пропущено %1").arg(delta - 1));
        }
    }
    m_lastSeq = seq;
}

void MavlinkManager::checkLink()
{
    const qint64 now = m_clock.elapsed();
    while (!m_recent.isEmpty() && m_recent.head() <= now - 1000)
        m_recent.dequeue();
    m_rate = m_recent.size();

    if (m_link == Online && now - m_lastTick >= 3000) {
        m_link = Lost;
        m_watchdog.stop();
        m_lastSeq = -1;
        emit event(tr("Связь"), tr("Связь потеряна: нет корректных пакетов 3 секунды"));
        emit stateChanged();
        emit linkLost();
    }
    emit stateChanged();
}

void MavlinkManager::sendCommand(Command command, double altitude)
{
    if (!canSend()) {
        emit event(tr("Команда"), tr("Отправка недоступна: нет связи или уже ожидается ACK"));
        return;
    }
    QByteArray payload(33, '\0');
    const float unchanged = std::numeric_limits<float>::quiet_NaN();
    putFloat(payload, 12, unchanged);
    putFloat(payload, 16, unchanged);
    putFloat(payload, 20, unchanged);
    putFloat(payload, 24, static_cast<float>(altitude));
    put<quint16>(payload, 28, command);
    put<quint8>(payload, 30, VehicleId);
    put<quint8>(payload, 31, ComponentId);

    const auto packet = MavlinkMessageParser::encode(CommandLong, m_seq++, StationId, payload);
    if (m_socket.writeDatagram(packet, QHostAddress::LocalHost, SimulatorPort) != packet.size()) {
        emit event(tr("Ошибка отправки"), m_socket.errorString());
        return;
    }
    m_pending = command;
    m_ackTimer.start();
    emit commandStatus(tr("WAITING_ACK — ожидание подтверждения"));
    const auto details = command == Takeoff ? tr("Взлёт на %1 м").arg(altitude) : tr("Посадка");
    emit event(tr("Отправлено"), details + tr(" — COMMAND_LONG (%1)").arg(command));
    emit stateChanged();
}

void MavlinkManager::applyFrame(const Frame &frame)
{
    const QByteArray &p = frame.payload;
    if (frame.id == Position) {
        m_model.setPosition(read<qint32>(p, 4) / 1e7, read<qint32>(p, 8) / 1e7,
                            read<qint32>(p, 16) / 1000.0, -read<qint16>(p, 24) / 100.0);
    } else if (frame.id == Heartbeat) {
        m_model.setMode(modeName(read<quint32>(p, 0)));
    } else if (frame.id == CommandAck) {
        const auto target = read<quint8>(p, 8);
        const auto component = read<quint8>(p, 9);
        if (!m_pending || read<quint16>(p, 0) != m_pending
            || (target != 0 && target != StationId) || (component != 0 && component != ComponentId))
            return;

        const quint8 result = read<quint8>(p, 2);
        switch (result) {
        case 0: finishCommand(tr("Команда принята")); break;
        case 1: finishCommand(tr("Команда временно отклонена")); break;
        case 2: finishCommand(tr("Команда отклонена")); break;
        case 3: finishCommand(tr("Команда не поддерживается")); break;
        case 4: finishCommand(tr("Ошибка выполнения: отказ сенсора")); break;
        case 5:
            emit commandStatus(tr("Команда выполняется — ожидается итоговый ACK"));
            emit event(tr("COMMAND_ACK"), tr("Команда выполняется"));
            break;
        default: finishCommand(tr("Неизвестный результат ACK (%1)").arg(result)); break;
        }
    }
}

void MavlinkManager::finishCommand(const QString &status)
{
    m_ackTimer.stop();
    m_pending = 0;
    emit commandStatus(status);
    emit event(tr("Команда"), status);
    emit stateChanged();
}
