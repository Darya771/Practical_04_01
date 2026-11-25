QT += widgets network
CONFIG += c++17
TEMPLATE = app
TARGET = UavSimulator
INCLUDEPATH += ../network
SOURCES += main.cpp uavsimulator.cpp ../network/mavlinkmessageparser.cpp
HEADERS += uavsimulator.h ../network/mavlinkmessageparser.h
win32-msvc*: QMAKE_CXXFLAGS += /utf-8
