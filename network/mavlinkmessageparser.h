#pragma once

#include <QByteArray>
#include <QString>
#include <QtGlobal>
#include <QtEndian>
#include <cstring>

namespace Mavlink {
constexpr quint16 StationPort = 14540;
constexpr quint16 SimulatorPort = 14551;
constexpr quint8 StationId = 255;
constexpr quint8 VehicleId = 1;
constexpr quint8 ComponentId = 1;

enum MessageId { Heartbeat = 0, Position = 33, CommandLong = 76, CommandAck = 77 };
enum Command { Land = 21, Takeoff = 22 };
enum Mode { Standby, Climbing, Holding, Descending, SensorFailure };

struct Frame {
    quint32 id = 0;
    quint8 seq = 0;
    quint8 system = 0;
    quint8 component = 0;
    QByteArray payload;
};

template<typename T>
T read(const QByteArray &payload, int offset)
{
    return qFromLittleEndian<T>(payload.constData() + offset);
}

template<typename T>
void put(QByteArray &payload, int offset, T value)
{
    qToLittleEndian<T>(value, payload.data() + offset);
}

inline float readFloat(const QByteArray &payload, int offset)
{
    const auto bits = read<quint32>(payload, offset);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

inline void putFloat(QByteArray &payload, int offset, float value)
{
    quint32 bits;
    std::memcpy(&bits, &value, sizeof(bits));
    put(payload, offset, bits);
}

class MavlinkMessageParser {
public:
    static QByteArray encode(quint32 id, quint8 seq, quint8 system, QByteArray payload);
    static bool decode(const QByteArray &bytes, Frame &frame, QString &error);
};

QString modeName(quint32 mode);
}
