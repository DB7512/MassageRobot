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
    void on_pushButton_clicked();

    void on_ConnectButton_clicked();

    void on_DisconnectButton_clicked();

    void on_EnableButton_clicked();

    void on_UnableButton_clicked();

    void on_MassageButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::Widget *ui;

public:

};
#endif // WIDGET_H
