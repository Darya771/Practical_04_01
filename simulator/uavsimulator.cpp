#include "uavsimulator.h"

#include <QNetworkDatagram>
#include <cmath>

using namespace Mavlink;

UavSimulator::UavSimulator(QObject *parent) : QObject(parent)
{
    m_timer.setInterval(100);
    m_timer.setTimerType(Qt::PreciseTimer);
    connect(&m_timer, &QTimer::timeout, this, &UavSimulator::tick);
    connect(&m_socket, &QUdpSocket::readyRead, this, &UavSimulator::receive);
}

bool UavSimulator::start()
{
    if (m_timer.isActive())
        return true;
    if (!m_socket.bind(QHostAddress::LocalHost, SimulatorPort, QUdpSocket::DontShareAddress)) {
        emit status(tr("Порт %1: %2").arg(SimulatorPort).arg(m_socket.errorString()));
        return false;
    }
    m_clock.start();
    m_previousTick = 0;
    m_heartbeatTick = -1000;
    m_timer.start();
    emit status(tr("Телеметрия передаётся на 127.0.0.1:14540"));
    return true;
}

void UavSimulator::stop()
{
    m_timer.stop();
    m_socket.close();
    m_skip = m_corruption = 0;
    emit status(tr("Передача остановлена"));
}

void UavSimulator::setSlow(bool slow)
{
    m_timer.setInterval(slow ? 500 : 100);
}

void UavSimulator::tick()
{
    const qint64 now = m_clock.elapsed();
    updateFlight((now - m_previousTick) / 1000.0);
    m_previousTick = now;
    emit sample(m_altitude, m_speed);

    if (m_skip > 0) {
        --m_skip;
        ++m_seq;
        return;
    }

    QByteArray position(28, '\0');
    put<quint32>(position, 0, static_cast<quint32>(now));
    put<qint32>(position, 4, 557522200);
    put<qint32>(position, 8, 376155600);
    put<qint32>(position, 12, qRound((150 + m_altitude) * 1000));
    put<qint32>(position, 16, qRound(m_altitude * 1000));
    put<qint16>(position, 24, static_cast<qint16>(qRound(-m_speed * 100)));
    put<quint16>(position, 26, 65535);
    send(Position, position);

    if (now - m_heartbeatTick < 1000)
        return;
    m_heartbeatTick = now;
    quint32 mode = m_altitude > 0 ? Holding : Standby;
    if (m_sensorFailure)
        mode = SensorFailure;
    else if (m_speed > 0)
        mode = Climbing;
    else if (m_speed < 0)
        mode = Descending;

    QByteArray heartbeat(9, '\0');
    put(heartbeat, 0, mode);
    heartbeat[4] = char(2); // MAV_TYPE_QUADROTOR
    heartbeat[5] = char(8); // MAV_AUTOPILOT_INVALID
    heartbeat[6] = char(1); // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    heartbeat[7] = char(m_sensorFailure ? 5 : 4);
    heartbeat[8] = char(3);
    send(Heartbeat, heartbeat);
}

void UavSimulator::updateFlight(double seconds)
{
    const double before = m_altitude;
    if (!m_sensorFailure) {
        if (m_descending)
            m_altitude = qMax(m_target, m_altitude - 4 * seconds);
        else
            m_altitude = qMin(m_target, m_altitude + 6 * seconds);
    }
    m_speed = seconds > 0 ? (m_altitude - before) / seconds : 0;
}

void UavSimulator::receive()
{
    while (m_socket.hasPendingDatagrams()) {
        const auto datagram = m_socket.receiveDatagram();
        Frame frame;
        QString error;
        if (!datagram.isValid() || datagram.senderAddress() != QHostAddress::LocalHost
            || datagram.senderPort() != StationPort
            || !MavlinkMessageParser::decode(datagram.data(), frame, error)
            || frame.id != CommandLong || frame.system != StationId || frame.component != ComponentId)
            continue;

        const QByteArray &p = frame.payload;
        if (read<quint8>(p, 30) != VehicleId || read<quint8>(p, 31) != ComponentId)
            continue;

        const auto command = read<quint16>(p, 28);
        const float altitude = readFloat(p, 24);
        quint8 result = 0;
        if (m_sensorFailure)
            result = 4;
        else if (command == Takeoff && std::isfinite(altitude) && altitude >= 10
                 && altitude <= 500 && altitude > m_altitude) {
            m_target = altitude;
            m_descending = false;
        } else if (command == Land) {
            m_target = 0;
            m_descending = true;
        } else {
            result = command == Takeoff ? 2 : 3;
        }

        if (m_ignoreAck) {
            emit status(tr("Команда %1 получена, ACK пропущен").arg(command));
            continue;
        }
        QByteArray ack(10, '\0');
        put(ack, 0, command);
        put(ack, 2, result);
        put(ack, 8, frame.system);
        put(ack, 9, frame.component);
        send(CommandAck, ack);
        emit status(tr("Отправлен COMMAND_ACK для команды %1, код %2").arg(command).arg(result));
    }
}

void UavSimulator::send(quint32 id, const QByteArray &payload)
{
    auto packet = MavlinkMessageParser::encode(id, m_seq++, VehicleId, payload);
    if (id == Position && m_corruption != 0) {
        if (m_corruption == 2)
            packet.chop(1);
        else
            packet[10] = char(packet.at(10) ^ 1);
        m_corruption = 0;
    }
    if (m_socket.writeDatagram(packet, QHostAddress::LocalHost, StationPort) != packet.size())
        emit status(tr("Ошибка отправки: %1").arg(m_socket.errorString()));
}
