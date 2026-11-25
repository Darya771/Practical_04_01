#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDoubleSpinBox>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <cmath>

class TelemetryPlot : public QWidget {
public:
    explicit TelemetryPlot(QWidget *parent) : QWidget(parent) { setMinimumHeight(235); }

    void append(double time, double altitude, double speed)
    {
        m_altitude.enqueue(QPointF(time, altitude));
        m_speed.enqueue(QPointF(time, speed));
        while (m_altitude.head().x() < time - 30) {
            m_altitude.dequeue();
            m_speed.dequeue();
        }
        update();
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), QColor("#f4f7fa"));
        const double finish = m_altitude.isEmpty() ? 30 : qMax(30.0, m_altitude.last().x());
        const double begin = finish - 30;
        double maxAltitude = 10;
        for (const auto &point : m_altitude)
            maxAltitude = qMax(maxAltitude, point.y());
        maxAltitude = std::ceil(maxAltitude / 10) * 10;

        const double half = height() / 2.0;
        drawSeries(painter, QRectF(49, 26, width() - 65, half - 45), m_altitude,
                   tr("Высота, м"), QColor("#1554ad"), 0, maxAltitude, begin, finish);
        drawSeries(painter, QRectF(49, half + 22, width() - 65, half - 48), m_speed,
                   tr("Вертикальная скорость, м/с"), QColor("#9e450b"), -6, 6, begin, finish);
    }

private:
    void drawSeries(QPainter &painter, const QRectF &area, const QQueue<QPointF> &series,
                    const QString &name, const QColor &color, double low, double high,
                    double begin, double finish)
    {
        painter.setPen(QColor("#23364a"));
        painter.drawText(QPointF(area.left(), area.top() - 8), name);
        for (int i = 0; i < 3; ++i) {
            const double y = area.top() + i * area.height() / 2;
            painter.setPen(QColor("#d5dce5"));
            painter.drawLine(QPointF(area.left(), y), QPointF(area.right(), y));
            painter.setPen(QColor("#23364a"));
            painter.drawText(QRectF(0, y - 8, area.left() - 6, 16),
                             Qt::AlignRight | Qt::AlignVCenter,
                             QString::number(high - i * (high - low) / 2, 'g', 4));
        }
        painter.drawText(QRectF(area.left(), area.bottom() + 2, area.width(), 16),
                         Qt::AlignLeft, tr("%1 с").arg(begin, 0, 'f', 0));
        painter.drawText(QRectF(area.left(), area.bottom() + 2, area.width(), 16),
                         Qt::AlignRight, tr("%1 с").arg(finish, 0, 'f', 0));
        QPainterPath path;
        bool first = true;
        QPointF previous;
        for (const auto &point : series) {
            const QPointF mapped(area.left() + (point.x() - begin) / (finish - begin) * area.width(),
                                 area.bottom() - (point.y() - low) / (high - low) * area.height());
            if (first || point.x() - previous.x() > 1)
                path.moveTo(mapped);
            else
                path.lineTo(mapped);
            previous = point;
            first = false;
        }
        painter.setPen(QPen(color, 2));
        painter.drawPath(path);
    }

    QQueue<QPointF> m_altitude;
    QQueue<QPointF> m_speed;
};

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), m_manager(m_model)
{
    Ui::MainWindow ui;
    ui.setupUi(this);
    resize(920, 725);
    m_elapsed.start();

    m_link = ui.labelLinkStatus;
    m_sender = ui.labelSender;
    m_packets = ui.labelPackets;
    m_times = ui.labelPacketTimes;
    m_coordinates = ui.labelCoordinates;
    m_altitude = ui.labelAltitude;
    m_speed = ui.labelVerticalSpeed;
    m_mode = ui.labelFlightMode;
    m_command = ui.labelCommandStatus;
    m_open = ui.btnStartLink;
    m_close = ui.btnStopLink;
    m_takeoff = ui.btnTakeoff;
    m_land = ui.btnLand;
    m_target = ui.spinTakeoffAltitude;
    m_log = ui.listEventLog;
    m_plot = new TelemetryPlot(ui.plotArea);
    ui.plotLayout->setContentsMargins(0, 0, 0, 0);
    ui.plotLayout->addWidget(m_plot);

    connect(m_open, &QPushButton::clicked, &m_manager, &MavlinkManager::open);
    connect(m_close, &QPushButton::clicked, &m_manager, &MavlinkManager::close);
    connect(m_takeoff, &QPushButton::clicked, this, [this] {
        if (QMessageBox::question(this, tr("Подтверждение"),
                                  tr("Начать имитацию взлёта на %1 м?").arg(m_target->value()),
                                  QMessageBox::Yes | QMessageBox::No, QMessageBox::No)
            == QMessageBox::Yes) {
            m_manager.sendCommand(Mavlink::Takeoff, m_target->value());
        }
    });
    connect(m_land, &QPushButton::clicked, this, [this] {
        m_manager.sendCommand(Mavlink::Land);
    });
    connect(&m_manager, &MavlinkManager::stateChanged, this, &MainWindow::refreshState);
    connect(&m_manager, &MavlinkManager::commandStatus, m_command, &QLabel::setText);
    connect(&m_manager, &MavlinkManager::event, this, &MainWindow::log);
    connect(&m_manager, &MavlinkManager::linkLost, this, [this] {
        if (m_warned)
            return;
        m_warned = true;
        QMessageBox::warning(this, tr("Потеря связи"), tr("Телеметрия от эмулятора перестала поступать."));
    });
    connect(&m_model, &TelemetryModel::positionChanged, this, &MainWindow::updatePosition);
    connect(&m_model, &TelemetryModel::modeChanged, this, [this] {
        m_mode->setText(m_model.mode());
        const bool failure = m_model.mode() == Mavlink::modeName(Mavlink::SensorFailure);
        m_mode->setStyleSheet(failure ? QStringLiteral("color: #aa2525; font-weight: bold;") : QString());
        log(failure ? tr("Авария") : tr("Режим"), m_model.mode());
    });
    refreshState();
}

void MainWindow::refreshState()
{
    QString name;
    switch (m_manager.link()) {
    case MavlinkManager::Closed: name = tr("Связь отсутствует"); break;
    case MavlinkManager::Waiting: name = tr("Ожидание пакетов"); break;
    case MavlinkManager::Online: name = tr("Связь установлена"); break;
    case MavlinkManager::Lost: name = tr("Связь потеряна"); break;
    }
    const double rate = m_manager.rate();
    const QString color = rate > 5 ? QStringLiteral("#21663b")
                        : rate >= 1 ? QStringLiteral("#856003")
                                    : QStringLiteral("#aa2525");
    m_link->setStyleSheet(QStringLiteral("color: %1; font-weight: bold;").arg(color));
    m_link->setText(tr("%1 · %2 пак./с").arg(name).arg(rate, 0, 'f', 0));
    m_sender->setText(m_manager.sender().isEmpty() ? tr("—") : m_manager.sender());
    m_packets->setText(tr("Принято %1 · потеряно %2 · ошибок %3")
                         .arg(m_manager.received()).arg(m_manager.missing()).arg(m_manager.invalid()));
    const auto showTime = [](const QDateTime &time) {
        return time.isValid() ? time.toString(QStringLiteral("HH:mm:ss")) : QStringLiteral("—");
    };
    m_times->setText(showTime(m_manager.firstPacket()) + QStringLiteral(" / ")
                     + showTime(m_manager.lastPacket()));
    const bool closed = m_manager.link() == MavlinkManager::Closed;
    m_open->setEnabled(closed);
    m_close->setEnabled(!closed);
    m_takeoff->setEnabled(m_manager.canSend());
    m_land->setEnabled(m_manager.canSend());
}

void MainWindow::updatePosition()
{
    m_coordinates->setText(QStringLiteral("%1° / %2°").arg(m_model.latitude(), 0, 'f', 6)
                                                         .arg(m_model.longitude(), 0, 'f', 6));
    m_altitude->setText(tr("%1 м").arg(m_model.altitude(), 0, 'f', 2));
    m_speed->setText(tr("%1 м/с").arg(m_model.verticalSpeed(), 0, 'f', 2));
    m_plot->append(m_elapsed.elapsed() / 1000.0, m_model.altitude(), m_model.verticalSpeed());
}

void MainWindow::log(const QString &type, const QString &message)
{
    m_log->addItem(QStringLiteral("[%1] %2 · %3").arg(QTime::currentTime().toString("HH:mm:ss"),
                                                      type, message));
    if (m_log->count() > 500)
        delete m_log->takeItem(0);
    m_log->scrollToBottom();
}
