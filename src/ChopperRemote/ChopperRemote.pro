# -------------------------------------------------
# Project created by QtCreator 2010-05-06T21:06:21
# -------------------------------------------------
TARGET = ChopperRemote
TEMPLATE = app
INCLUDEPATH += /usr/include/qwt-qt4
SOURCES += main.cpp \
    joystick.cpp \
    ProximityRadar.cpp \
    Communication.cpp \
    Attitude.cpp \
    mainwindow.cpp
HEADERS += main.h \
    ProximityRadar.h \
    main.h \
    joystick.h \
    Communication.h \
    Attitude.h \
    mainwindow.h
FORMS += mainwindow.ui
LIBS += -L/usr/lib \
    -lboost_system-mt \
    -lboost_thread-mt \
    -lqwt-qt4
