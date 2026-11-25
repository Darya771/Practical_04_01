#include "uavsimulator.h"

#include <QApplication>
#include <QCheckBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QWidget window;
    window.setWindowTitle(QObject::tr("Практика 4 · Эмулятор БПЛА"));
    window.resize(565, 355);
    UavSimulator simulator;

    auto *layout = new QVBoxLayout(&window);
    layout->setContentsMargins(16, 14, 16, 14);
    layout->addWidget(new QLabel(QObject::tr("UDP 127.0.0.1:14551 → 127.0.0.1:14540")));
    auto *controls = new QHBoxLayout;
    auto *start = new QPushButton(QObject::tr("Запустить БПЛА"));
    auto *stop = new QPushButton(QObject::tr("Отключить БПЛА"));
    controls->addWidget(start);
    controls->addWidget(stop);
    layout->addLayout(controls);

    auto *simulation = new QGroupBox(QObject::tr("Имитация сбоев"));
    auto *faults = new QVBoxLayout(simulation);
    auto *slow = new QCheckBox(QObject::tr("Снизить телеметрию до 2 Гц"));
    auto *ignore = new QCheckBox(QObject::tr("Не отправлять COMMAND_ACK"));
    auto *failure = new QCheckBox(QObject::tr("Отказ сенсора"));
    auto *drop = new QPushButton(QObject::tr("Пропустить два пакета"));
    auto *corrupt = new QPushButton(QObject::tr("Испортить CRC следующего пакета"));
    auto *truncate = new QPushButton(QObject::tr("Обрезать следующий пакет"));
    faults->addWidget(slow);
    faults->addWidget(ignore);
    faults->addWidget(failure);
    faults->addWidget(drop);
    faults->addWidget(corrupt);
    faults->addWidget(truncate);
    layout->addWidget(simulation);

    auto *details = new QFormLayout;
    auto *telemetry = new QLabel(QObject::tr("—"));
    auto *status = new QLabel(QObject::tr("Передача остановлена"));
    status->setWordWrap(true);
    details->addRow(QObject::tr("Высота и скорость"), telemetry);
    details->addRow(QObject::tr("Событие"), status);
    layout->addLayout(details);

    const auto showRunning = [=](bool running) {
        start->setEnabled(!running);
        stop->setEnabled(running);
        drop->setEnabled(running);
        corrupt->setEnabled(running);
        truncate->setEnabled(running);
    };
    showRunning(false);
    QObject::connect(start, &QPushButton::clicked, &simulator, [&] {
        if (simulator.start())
            showRunning(true);
    });
    QObject::connect(stop, &QPushButton::clicked, &simulator, [&] {
        simulator.stop();
        showRunning(false);
    });
    QObject::connect(slow, &QCheckBox::toggled, &simulator, &UavSimulator::setSlow);
    QObject::connect(ignore, &QCheckBox::toggled, &simulator, &UavSimulator::setIgnoreAck);
    QObject::connect(failure, &QCheckBox::toggled, &simulator, &UavSimulator::setSensorFailure);
    QObject::connect(drop, &QPushButton::clicked, &simulator, &UavSimulator::skipTwo);
    QObject::connect(corrupt, &QPushButton::clicked, &simulator, [&] { simulator.corrupt(false); });
    QObject::connect(truncate, &QPushButton::clicked, &simulator, [&] { simulator.corrupt(true); });
    QObject::connect(&simulator, &UavSimulator::status, status, &QLabel::setText);
    QObject::connect(&simulator, &UavSimulator::sample, telemetry, [telemetry](double h, double v) {
        telemetry->setText(QObject::tr("%1 м · %2 м/с").arg(h, 0, 'f', 2).arg(v, 0, 'f', 2));
    });

    window.show();
    return app.exec();
}
