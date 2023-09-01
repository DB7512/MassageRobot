#include "tcpserver.h"
#include "robotcontrol.h"
TcpServer::TcpServer(QObject *parent)
    : QObject{parent}
{
//    qDebug("2");
    m_tcpserver = new QTcpServer(this);
    m_tcpserver->listen(QHostAddress::AnyIPv4,30003);
    //有新的连接时触发
    connect(m_tcpserver,SIGNAL(newConnection()),this,SLOT(onNewConnection()));
    //Feedback robot status
    m_sendtimer = new QTimer(this);
    m_sendtimer->setInterval(300);
    connect(m_sendtimer,SIGNAL(timeout()),this,SLOT(onSocketReadySend()));
}

TcpServer::~TcpServer()
{
    if(m_tcpserver) {
        delete m_tcpserver;
        m_tcpserver = NULL;
    }
}

void TcpServer::InitSocket()
{
    return;
}

void TcpServer::ParseData(Instruction &instruction, QString &msg)
{
    if(!msg.contains("(",Qt::CaseSensitive)||!msg.contains(")",Qt::CaseSensitive)) return;
    QStringList msg_list = msg.remove(QRegExp("\\s")).split("(");//去除空格
    qDebug()<<"msg"<<msg;
    if(msg_list[0] == "connect") {
        instruction.instruction_type = CONNECTION;
    }else if(msg_list[0] == "disconnect") {
        instruction.instruction_type = DISCONNECTION;
    }else if(msg_list[0] == "enable") {
        instruction.instruction_type = ENABLE;
    }else if(msg_list[0] == "disable") {
        instruction.instruction_type = DISENALE;
    }else if(msg_list[0] == "massage") {
        instruction.instruction_type = MASSAGE;
    }else if(msg_list[0] == "reset") {
        instruction.instruction_type = RESET;
    }else if(msg_list[0] == "movel") {
        instruction.instruction_type = MOVEL;
    }else if(msg_list[0] == "movej") {
        instruction.instruction_type = MOVEJ;
    }else {
        return;
    }
    if(instruction.instruction_type == MASSAGE) {
        if(!m_massagestatus) {
            vector<float> position(3);
            QStringList msg_data_list = msg_list[1].split(")")[0].split(";");
            int acupoint_number = msg_data_list.size();
            for (int var = 0; var < acupoint_number; var++) {
                QStringList msg_point_list = msg_data_list[var].split(",");
                for (int i = 0; i < 3; i++) {
                    position[i] = msg_point_list[i].toFloat();
                }
                m_acupoint.push_back(position);
            }
        }
    }
}

void TcpServer::SendInstruction(Instruction &instruction)
{
    switch (instruction.instruction_type) {
    case CONNECTION:
        if(!m_connectstatus) {
            qDebug("connect");
            emit connection();
        }
        break;
    case DISCONNECTION:
        if(m_connectstatus) {
            qDebug("disconnect");
            emit disconnection();
        }
        break;
    case ENABLE:
        if(m_enablestatus == false && m_connectstatus == true) {
            qDebug("enable");
            emit enableRobot();
        }
        break;
    case DISENALE:
        if(m_connectstatus == true && m_enablestatus == true) {
            qDebug("disable");
            emit disableRobot();
        }
        break;
    case MASSAGE:
        if(m_massagestatus == false && m_connectstatus == true && m_enablestatus == true) {
            qDebug("massage");
            emit startMassage(m_acupoint);
            vector<vector<float> > ().swap(m_acupoint);
        }else {
            vector<vector<float> > ().swap(m_acupoint);
        }
        break;
    case RESET:
        if(m_connectstatus == true && m_enablestatus == true && m_massagestatus == false) {
            qDebug("reset");
            emit resetRobot();
        }
        break;
    case MOVEL:

        break;
    case MOVEJ:

        break;
    case NOTYPE:

        break;
    default:
        break;
    }
}

void TcpServer::GetRobotStatus(RobotStatus &robotstatus)
{
    robotstatus.connect_status = m_connectstatus;
    robotstatus.enable_status = m_enablestatus;
    robotstatus.massage_status = m_massagestatus;
}

void TcpServer::SendStatusTransformation(QByteArray &status, RobotStatus &robotstatus)
{
    char pstatus[sizeof(struct RobotStatus)];
    memcpy(pstatus, &robotstatus, sizeof(struct RobotStatus));
//    char *pstate = pstatus;
    memcpy(status.data(),pstatus,sizeof(pstatus));
    qDebug("pstatus");
}

void TcpServer::Connection()
{
//    GetRobotControlInstance().ConnectRobot();
}

void TcpServer::Disconnection()
{
//    GetRobotControlInstance().DisconnectRobot();
}

void TcpServer::Enable()
{
//    GetRobotControlInstance().EnableRobot();
}

void TcpServer::Disable()
{
//    GetRobotControlInstance().DisableRobot();
}

void TcpServer::Massage()
{
//    GetRobotControlInstance()->SetMassage();
//    if(GetRobotControlInstance()->m_arm->is_connected()) {
//        GetRobotControlInstance()->m_massagestate = true;
//        GetRobotControlInstance()->MassagePerform(GetRobotControlInstance()->m_arm);
//        GetRobotControlInstance()->RobotMassage();
//    }
}

void TcpServer::Reset()
{
//    GetRobotControlInstance().ResetRobot();
}

void TcpServer::Movel()
{

}

void TcpServer::Movej()
{

}

void TcpServer::onNewConnection()
{
    m_sendtimer->start();
    m_tcpsocket = m_tcpserver->nextPendingConnection();
    QObject::connect(m_tcpsocket,SIGNAL(readyRead()),this,SLOT(onSocketReadyRead()));
}

void TcpServer::onClientConnected()
{
    m_connectstatus = true;
}

void TcpServer::onClientDisconnected()
{
    m_connectstatus = false;
    m_tcpsocket->deleteLater();
}

void TcpServer::onSocketReadyRead()
{
    QByteArray msg_array = m_tcpsocket->readAll();
    QString msg_string = QString(msg_array);
    if (msg_string == nullptr || msg_string.size() == 0) return;
    //构建指令结构体
    Instruction instruction;
    //解析收到的数据
    ParseData(instruction, msg_string);
    //下发运动数据
    SendInstruction(instruction);
}

void TcpServer::onSocketReadySend()
{
    if(m_tcpserver == nullptr) return;
    RobotStatus robotstatus;
    GetRobotStatus(robotstatus);
    QByteArray status;
    status.append(reinterpret_cast<char *>(&robotstatus), sizeof(RobotStatus));
    char pstatus[sizeof(struct RobotStatus)];
    memcpy(pstatus, &robotstatus, sizeof(struct RobotStatus));
//    SendStatusTransformation(status,robotstatus);
    int ret = m_tcpsocket->write((char*)&robotstatus,sizeof(robotstatus));
//    int ret = m_tcpsocket->write(status);
    if(ret == -1) {
        if (m_tcpsocket != nullptr) {
            m_tcpsocket->close();
            delete m_tcpsocket;
            m_tcpsocket = nullptr;
            m_sendtimer->stop();
        }
    }else {
        m_tcpsocket->flush();
    }
}

void TcpServer::connectStatus(bool status)
{
    m_connectstatus = status;
}

void TcpServer::enableStatus(bool status)
{
    m_enablestatus = status;
}

void TcpServer::massageStatus(bool status)
{
    m_massagestatus = status;
}
