#ifndef TCPSERVER_H
#define TCPSERVER_H

#include "qdatetime.h"
#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>

using namespace std;

class RobotControl;

//socket通信指令类型
typedef enum {
    CONNECTION,
    DISCONNECTION,
    ENABLE,
    DISENALE,
    MASSAGE,
    RESET,
    MOVEL,
    MOVEJ,
    NOTYPE,
}SOCKETINSTRUCTIONTYPE;

//指令结构体
struct Instruction {
    SOCKETINSTRUCTIONTYPE instruction_type = NOTYPE;
    int acupoint_number = 0;
    vector<vector<float> > acupoint_information;
};

//socket通信数据结构体
struct RobotStatus {
    int connect_status;
    int enable_status;
    int massage_status;
};

class TcpServer : public QObject
{
    Q_OBJECT
public:
    explicit TcpServer(QObject *parent = nullptr);
    ~TcpServer();

public:
    void InitSocket();
    void ParseData(Instruction &instruction, QString &msg);
    void SendInstruction(Instruction &instruction);
    void GetRobotStatus(RobotStatus &robotstatus);
    void SendStatusTransformation(QByteArray &status, RobotStatus &robotstatus);

    void Connection();
    void Disconnection();
    void Enable();
    void Disable();
    void Massage();
    void Reset();
    void Movel();
    void Movej();

    QTcpSocket* m_tcpsocket;
    QTcpServer* m_tcpserver;
    RobotStatus m_robotstatus;
    QTimer *m_sendtimer;
    int m_connectstatus;
    int m_enablestatus;
    int m_massagestatus;
    vector<vector<float> > m_acupoint;
signals:
    void connection();
    void disconnection();
    void enableRobot();
    void disableRobot();
    void startMassage(vector<vector<float> > acupoint);
    void resetRobot();

public slots:
    void onNewConnection();
    void onClientConnected();
    void onClientDisconnected();
    void onSocketReadyRead();
    void onSocketReadySend();
    void connectStatus(bool status);
    void enableStatus(bool status);
    void massageStatus(bool status);
};

static TcpServer* GetTcpServerInstance()
{
    static TcpServer TcpServerInstance;
    return &TcpServerInstance;
}

#endif // TCPSERVER_H
