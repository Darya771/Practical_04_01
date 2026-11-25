#pragma once

#include "mavlinkmessageparser.h"
#include "telemetrymodel.h"
#include <QDateTime>
#include <QElapsedTimer>
#include <QQueue>
#include <QTimer>
#include <QUdpSocket>

class MavlinkManager : public QObject {
    Q_OBJECT
public:
    enum Link { Closed, Waiting, Online, Lost };

    explicit MavlinkManager(TelemetryModel &model, QObject *parent = nullptr);
    void open();
    void close();
    void sendCommand(Mavlink::Command command, double altitude = 0);
    Link link() const { return m_link; }
    bool canSend() const { return m_link == Online && m_pending == 0; }
    double rate() const { return m_rate; }
    quint64 received() const { return m_received; }
    quint64 missing() const { return m_missing; }
    quint64 invalid() const { return m_invalid; }
    QString sender() const { return m_sender; }
    QDateTime lastPacket() const { return m_lastPacket; }
    QDateTime firstPacket() const { return m_firstPacket; }

signals:
    void stateChanged();
    void commandStatus(const QString &status);
    void event(const QString &type, const QString &message);
    void linkLost();

private:
    void receive();
    void checkLink();
    void checkSequence(quint8 seq);
    void applyFrame(const Mavlink::Frame &frame);
    void finishCommand(const QString &status);

    TelemetryModel &m_model;
    QUdpSocket m_socket;
    QTimer m_linkTimer;
    QTimer m_watchdog;
    QTimer m_ackTimer;
    QElapsedTimer m_clock;
    QQueue<qint64> m_recent;
    QDateTime m_firstPacket;
    QDateTime m_lastPacket;
    qint64 m_lastTick = 0;
    QString m_sender;
    Link m_link = Closed;
    double m_rate = 0;
    quint64 m_received = 0;
    quint64 m_missing = 0;
    quint64 m_invalid = 0;
    int m_lastSeq = -1;
    quint8 m_seq = 0;
    quint16 m_pending = 0;
};
