#-------------------------------------------------
#
# Project created by QtCreator 2013-08-12T22:20:06
#
#-------------------------------------------------

QT       += core gui

TARGET = fysik1
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    physicsengine.cpp \
    mechanics2d.cpp \
    pendlum.cpp \
    enginesettings.cpp \
    pendlumdialog.cpp \
    connections.cpp \
    friction.cpp

HEADERS  += mainwindow.h \
    physicsengine.h \
    mechanics2d.h \
    pendlum.h \
    enginesettings.h \
    pendlumdialog.h \
    connections.h \
    friction.h

FORMS    += mainwindow.ui \
    enginesettings.ui \
    pendlumdialog.ui

LIBS += -larmadillo
