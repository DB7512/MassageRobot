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


void Widget::on_pushButton_clicked()
{
    //    m_tcpserver->m_tcpServer->listen(QHostAddress::AnyIPv4,30003);
    float start_point[6] = {-237.392,-119.023,452.799,0,0,0};
    float end_point[6] = {-645.021,-119.126,453.208,0,0,0};
    vector<vector<float> >interpolation_result;//插值点
    vector<vector<float> >interpolation_inf;
    GetRobotControlInstance().MoveL_old(100, start_point, end_point, 0.4, 0.6, 80, interpolation_result, interpolation_inf);
//    GetRobotControlInstance().MoveL(100, start_point, end_point, 0.2, 0.6, 80, interpolation_result, interpolation_inf);
    int a = 0;
    a = 1;
}


void Widget::on_ConnectButton_clicked()
{
//    while(!GetRobotControlInstance()->m_connectstate) {
//        GetRobotControlInstance()->ConnectRobot();
//        sleep_milliseconds(500);
//    }
//    if(GetRobotControlInstance()->Isconnected()) {
//        ui->ConnectButton->setEnabled(false);
//        ui->DisconnectButton->setEnabled(true);
//        ui->EnableButton->setEnabled(true);
//        ui->DisconnectButton->setEnabled(true);
//        ui->UnableButton->setEnabled(false);
//        ui->MassageButton->setEnabled(false);
//    }
}


void Widget::on_DisconnectButton_clicked()
{
//    while(GetRobotControlInstance()->m_connectstate) {
//        GetRobotControlInstance()->DisconnectRobot();
//        sleep_milliseconds(500);
//    }
//    if(!GetRobotControlInstance()->Isconnected() && (!GetRobotControlInstance()->m_connectstate)) {
//        ui->ConnectButton->setEnabled(false);
//        ui->DisconnectButton->setEnabled(false);
//        ui->EnableButton->setEnabled(false);
//        ui->UnableButton->setEnabled(false);
//        ui->MassageButton->setEnabled(false);
//    }
}


void Widget::on_EnableButton_clicked()
{
//    while(!GetRobotControlInstance()->m_enablestate) {
//        GetRobotControlInstance()->EnableRobot();
//        sleep_milliseconds(500);
//    }
//    if(GetRobotControlInstance()->Isconnected() && GetRobotControlInstance()->m_enablestate) {
//        ui->EnableButton->setEnabled(false);
//        ui->DisconnectButton->setEnabled(true);
//        ui->UnableButton->setEnabled(true);
//        ui->MassageButton->setEnabled(true);
//    }
}


void Widget::on_UnableButton_clicked()
{
//    while(GetRobotControlInstance()->m_enablestate) {
//        GetRobotControlInstance()->DisableRobot();
//        sleep_milliseconds(500);
//    }
//    if(GetRobotControlInstance()->Isconnected() && (!GetRobotControlInstance()->m_enablestate)) {
//        ui->EnableButton->setEnabled(true);
//        ui->DisconnectButton->setEnabled(true);
//        ui->UnableButton->setEnabled(false);
//        ui->MassageButton->setEnabled(true);
//    }
}


void Widget::on_MassageButton_clicked()
{
//    while(!GetRobotControlInstance()->m_massagestate) {
//        if(GetRobotControlInstance()->SetMassage()) {
//            sleep_milliseconds(500);
//            break;
//        }
//    }
//    if(GetRobotControlInstance()->Isconnected() && GetRobotControlInstance()->m_massagestate) {
//        ui->EnableButton->setEnabled(false);
//        ui->DisconnectButton->setEnabled(true);
//        ui->UnableButton->setEnabled(true);
//        ui->MassageButton->setEnabled(false);
//    }
}

#include <QTextStream>
#include <QFile>
void Widget::on_pushButton_2_clicked()
{
    float start_point[6] = {0,0,0,0,0,0};
    float end_point[6] = {0,0,50,0,0,0};
    float jmax = 80.0;
    vector<vector<float> >interpolation_result;//插值点
    vector<vector<float> >interpolation_inf;
    QFile data("data_old.txt");
    if(!data.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) return;
    data.close();
    QFile file("data_inf.txt");
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) return;
    file.close();
    GetRobotControlInstance().number = 0;
    for(int i = 0; i < 40; i++) {
        float vmax = 0.1;
        for(int j = 0; j < 20; j++) {
            float amax = 0.2;
            for(int k = 0; k < 30; k++) {
                GetRobotControlInstance().number += 1;
                QFile data("data_inf.txt");
                if(!data.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append)) return;
                QTextStream stream(&data);
                stream<<GetRobotControlInstance().number<<" pose "<<end_point[2]<<" v "<<vmax<<" a "<<amax<<"\n";
                data.close();
//                GetRobotControlInstance().MoveL_old(100, start_point, end_point, vmax, amax, 80, interpolation_result, interpolation_inf);
                GetRobotControlInstance().MoveL(100, start_point, end_point, vmax, amax, 80, interpolation_result, interpolation_inf);
                amax += 0.2;
            }
            vmax += 0.1;
        }
        end_point[2] += 50.0;
    }
    int a = 0;
    a = 1;
}

