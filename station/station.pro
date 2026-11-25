QT += widgets network
CONFIG += c++17
TEMPLATE = app
TARGET = GroundStation
INCLUDEPATH += ../network
SOURCES += main.cpp mainwindow.cpp mavlinkmanager.cpp ../network/mavlinkmessageparser.cpp
HEADERS += mainwindow.h mavlinkmanager.h telemetrymodel.h ../network/mavlinkmessageparser.h
win32-msvc*: QMAKE_CXXFLAGS += /utf-8

FORMS += mainwindow.ui
