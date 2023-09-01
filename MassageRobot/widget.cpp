#include "widget.h"
#include "ui_widget.h"
#include <QTcpServer>
#include <QThread>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);

    QThread *robotthread = new QThread();
    m_robotcontrol = new RobotControl;
    m_robotcontrol->moveToThread(robotthread);
    connect(robotthread,SIGNAL(started()),m_robotcontrol,SLOT(startThreadSlot()));
    robotthread->start();
}

Widget::~Widget()
{
    delete ui;
}

