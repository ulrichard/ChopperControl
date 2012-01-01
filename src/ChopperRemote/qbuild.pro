# this is the project file to build ChopperRemote for qtmoko
# qtmoko is a qtopia / debian based distor for the openmoko freerunner smartphone.

TEMPLATE=app
TARGET=ChopperRemote

CONFIG+=qtopia
DEFINES+=QTOPIA

# I18n info
STRING_LANGUAGE=en_US
LANGUAGES=en_US

# Package info
pkg [
    name=ChopperRemote
    desc="RC helicopter"
    license=GPLv3
    version=1.0
    maintainer="Richard Ulrich <richi@paraeasy.ch>"
]

# Input files
HEADERS=\
    main.h \
    ProximityRadar.h \
    main.h \
    joystick.h \
    Communication.h \
    Attitude.h \
    mainwindow.h

SOURCES=\
    main.cpp \
    joystick.cpp \
    ProximityRadar.cpp \
    Communication.cpp \
    Attitude.cpp \
    mainwindow.cpp

FORMS += mainwindow.ui

# Install rules
target [
    hint=sxe
    domain=untrusted
]

desktop [
    hint=desktop
    files=ChopperRemote.desktop
    path=/apps/Applications
]

#pics [
#    hint=pics
#    files=pics/*
#    path=/pics/qmplayer
#]

