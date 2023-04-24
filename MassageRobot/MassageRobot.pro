TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
        robotcontrol.cpp

HEADERS += \
    robotcontrol.h

unix:!macx: LIBS += -L$$PWD/../ -lxarm

INCLUDEPATH += $$PWD/../
DEPENDPATH += $$PWD/../

unix:!macx: LIBS += -L$$PWD/../../../../lib/x86_64-linux-gnu/ -lpthread

INCLUDEPATH += $$PWD/../../../../lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../lib/x86_64-linux-gnu
