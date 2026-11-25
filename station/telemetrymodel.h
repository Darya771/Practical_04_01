#pragma once

#include <QObject>
#include <QString>

class TelemetryModel : public QObject {
    Q_OBJECT
public:
    using QObject::QObject;
    double latitude() const { return m_latitude; }
    double longitude() const { return m_longitude; }
    double altitude() const { return m_altitude; }
    double verticalSpeed() const { return m_verticalSpeed; }
    QString mode() const { return m_mode; }

    void setPosition(double latitude, double longitude, double altitude, double verticalSpeed)
    {
        m_latitude = latitude;
        m_longitude = longitude;
        m_altitude = altitude;
        m_verticalSpeed = verticalSpeed;
        emit positionChanged();
    }

    void setMode(const QString &mode)
    {
        if (mode == m_mode)
            return;
        m_mode = mode;
        emit modeChanged();
    }

signals:
    void positionChanged();
    void modeChanged();

private:
    double m_latitude = 0;
    double m_longitude = 0;
    double m_altitude = 0;
    double m_verticalSpeed = 0;
    QString m_mode;
};
