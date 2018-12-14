TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    laserscanner.cpp \
    imageprocess.cpp \
    brushfire.cpp \
    movement.cpp \
    fuzzyobstacleavoidance.cpp \
    qlearning.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv


INCLUDEPATH += ../../fuzzylite-6.0/fuzzylite

LIBS += -L../../fuzzylite-6.0/release/bin -lfuzzylite-static

HEADERS += \
    laserscanner.h \
    imageprocess.h \
    brushfire.h \
    movement.h \
    fuzzyobstacleavoidance.h \
    qlearning.h

DISTFILES += \
    fuzzyobstacleavoidance.fll
