#pragma once

#include "mavlinkmessageparser.h"
#include <QElapsedTimer>
#include <QTimer>
#include <QUdpSocket>

class UavSimulator : public QObject {
    Q_OBJECT
public:
    explicit UavSimulator(QObject *parent = nullptr);
    bool start();
    void stop();
    void setSlow(bool slow);
    void skipTwo() { m_skip = 2; }
    void corrupt(bool header) { m_corruption = header ? 2 : 1; }
    void setIgnoreAck(bool enabled) { m_ignoreAck = enabled; }
    void setSensorFailure(bool enabled) { m_sensorFailure = enabled; }

signals:
    void status(const QString &text);
    void sample(double altitude, double speed);

private:
    void tick();
    void receive();
    void updateFlight(double seconds);
    void send(quint32 id, const QByteArray &payload);

    QUdpSocket m_socket;
    QTimer m_timer;
    QElapsedTimer m_clock;
    quint8 m_seq = 0;
    int m_skip = 0;
    int m_corruption = 0;
    bool m_ignoreAck = false;
    bool m_sensorFailure = false;
    bool m_descending = false;
    double m_altitude = 0;
    double m_target = 0;
    double m_speed = 0;
    qint64 m_previousTick = 0;
    qint64 m_heartbeatTick = -1000;
};
