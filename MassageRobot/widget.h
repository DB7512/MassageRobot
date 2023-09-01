#ifndef WIDGET_H
#define WIDGET_H

#include "robotcontrol.h"
#include <QWidget>
#include "usbcan.h"
#include <QTcpServer>
#include <QTcpSocket>
QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

    RobotControl *m_robotcontrol;

private slots:


private:
    Ui::Widget *ui;

public:

};
#endif // WIDGET_H
