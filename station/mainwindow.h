#pragma once

#include "mavlinkmanager.h"
#include "telemetrymodel.h"
#include <QElapsedTimer>
#include <QMainWindow>

class QLabel;
class QPushButton;
class QListWidget;
class QDoubleSpinBox;
class TelemetryPlot;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);

private:
    void refreshState();
    void updatePosition();
    void log(const QString &type, const QString &message);

    TelemetryModel m_model;
    MavlinkManager m_manager;
    QElapsedTimer m_elapsed;
    bool m_warned = false;
    QLabel *m_link;
    QLabel *m_sender;
    QLabel *m_packets;
    QLabel *m_times;
    QLabel *m_coordinates;
    QLabel *m_altitude;
    QLabel *m_speed;
    QLabel *m_mode;
    QLabel *m_command;
    QPushButton *m_open;
    QPushButton *m_close;
    QPushButton *m_takeoff;
    QPushButton *m_land;
    QDoubleSpinBox *m_target;
    QListWidget *m_log;
    TelemetryPlot *m_plot;
};
