QT       += core gui
QT       += network
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    robotcontrol.cpp \
    tcpserver.cpp \
    usbcan.cpp \
    widget.cpp

HEADERS += \
    controlcan.h \
    robotcontrol.h \
    tcpserver.h \
    usbcan.h \
    widget.h

FORMS += \
    widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

INCLUDEPATH += $$PWD/../
INCLUDEPATH += \home\hua\IOROBOT\eigen-3.4.0
DEPENDPATH += $$PWD/../

unix:!macx: LIBS += -L$$PWD/./ -lcontrolcan

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.

unix:!macx: LIBS += -L$$PWD/../ -lxarm

INCLUDEPATH += $$PWD/../
DEPENDPATH += $$PWD/../
