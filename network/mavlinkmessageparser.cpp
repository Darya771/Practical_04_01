#include "mavlinkmessageparser.h"

namespace Mavlink {
namespace {
int payloadSize(quint32 id)
{
    switch (id) {
    case Heartbeat: return 9;
    case Position: return 28;
    case CommandLong: return 33;
    case CommandAck: return 10;
    default: return 0;
    }
}

quint8 crcExtra(quint32 id)
{
    switch (id) {
    case Heartbeat: return 50;
    case Position: return 104;
    case CommandLong: return 152;
    case CommandAck: return 143;
    default: return 0;
    }
}

quint16 crc(const QByteArray &bytes, int end, quint32 id)
{
    quint16 value = 0xffff;
    const auto accumulate = [&value](quint8 byte) {
        quint8 mixed = byte ^ (value & 0xff);
        mixed ^= mixed << 4;
        value = (value >> 8) ^ (mixed << 8) ^ (mixed << 3) ^ (mixed >> 4);
    };
    for (int i = 1; i < end; ++i)
        accumulate(static_cast<quint8>(bytes.at(i)));
    accumulate(crcExtra(id));
    return value;
}
}

QByteArray MavlinkMessageParser::encode(quint32 id, quint8 seq, quint8 system,
                                      QByteArray payload)
{
    while (payload.size() > 1 && payload.endsWith('\0'))
        payload.chop(1);

    QByteArray bytes(10, '\0');
    bytes[0] = char(0xfd);
    bytes[1] = char(payload.size());
    bytes[4] = char(seq);
    bytes[5] = char(system);
    bytes[6] = char(ComponentId);
    bytes[7] = char(id & 0xff);
    bytes[8] = char((id >> 8) & 0xff);
    bytes[9] = char((id >> 16) & 0xff);
    bytes.append(payload);
    const quint16 checksum = crc(bytes, bytes.size(), id);
    bytes.append(char(checksum & 0xff));
    bytes.append(char(checksum >> 8));
    return bytes;
}

bool MavlinkMessageParser::decode(const QByteArray &bytes, Frame &frame, QString &error)
{
    if (bytes.size() < 12 || read<quint8>(bytes, 0) != 0xfd) {
        error = QStringLiteral("Повреждённый заголовок MAVLink 2");
        return false;
    }

    const int length = read<quint8>(bytes, 1);
    const quint32 id = read<quint8>(bytes, 7)
                     | (read<quint8>(bytes, 8) << 8)
                     | (read<quint8>(bytes, 9) << 16);
    if (bytes.size() != length + 12 || length < 1 || length > payloadSize(id)
        || read<quint8>(bytes, 2) != 0) {
        error = QStringLiteral("Некорректная длина, флаги или тип пакета");
        return false;
    }
    if (read<quint16>(bytes, length + 10) != crc(bytes, length + 10, id)) {
        error = QStringLiteral("Неверная контрольная сумма CRC");
        return false;
    }

    frame = {id, read<quint8>(bytes, 4), read<quint8>(bytes, 5),
             read<quint8>(bytes, 6), bytes.mid(10, length)};
    frame.payload.append(QByteArray(payloadSize(id) - length, '\0'));
    return true;
}

QString modeName(quint32 mode)
{
    switch (mode) {
    case Standby: return QStringLiteral("Ожидание");
    case Climbing: return QStringLiteral("Набор высоты");
    case Holding: return QStringLiteral("Удержание");
    case Descending: return QStringLiteral("Снижение");
    case SensorFailure: return QStringLiteral("Отказ сенсора");
    default: return QStringLiteral("Неизвестный режим");
    }
}
}
