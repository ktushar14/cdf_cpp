#-------------------------------------------------
#
# Project created by QtCreator 2021-01-17T00:08:32
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = coverage-search
TEMPLATE = app

CONFIG += c++14

INCLUDEPATH += include/

SOURCES += \
    src/dialog.cpp \
    src/robot_item.cpp \
    src/planner.cpp \
    src/wastar.cpp \
    src/xyenv.cpp \
    tests/main.cpp

HEADERS += \
    include/covsearch/robot_space.hpp \
    include/covsearch/planner.hpp \
    include/covsearch/types.hpp \
    include/covsearch/wastar.hpp \
    include/covsearch/xyenv.hpp \
    include/qt/dialog.h \
    include/qt/robot_item.hpp

FORMS += \
    src/dialog.ui
