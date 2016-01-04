#-------------------------------------------------
#
# Project created by QtCreator 2015-12-25T12:56:48
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = InverseDenavit
TEMPLATE = app
CONFIG += c++11

SOURCES += main.cpp\
        mainwindow.cpp \
    joint.cpp \
    figure.cpp

HEADERS  += mainwindow.h \
    joint.h \
    figure.h

FORMS    += mainwindow.ui

LIBS += -lmath
