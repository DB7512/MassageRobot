#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <QObject>
#include <stdlib.h>
#include <string>
#include <vector>
#include "tcpserver.h"
#include "usbcan.h"
#include "xarm/wrapper/xarm_api.h"
#include <thread>
#include <eigen-3.4.0/Eigen/Dense>

using namespace std;
using namespace Eigen;

class TcpServer;
class USBCAN;
typedef enum {
    TraUniTra = 0,
    TriUniTri,
    TraTra,
    TriTri,
}TrajectoryType;

class RobotControl : public QObject
{
    Q_OBJECT
public:
    explicit RobotControl(QObject *parent = nullptr);
    ~RobotControl();

public:
    //
    void ConnectRobot();
    void DisconnectRobot();
    bool Isconnected();
    bool Isenabled();
    void EnableRobot();
    void DisableRobot();
    void ResetRobot();
    void MassagePerform();
    void getForceData(float data);
    void setConnectStatus(bool status);
    void setEnableStatus(bool status);
    void setMassageStatus(bool status);
    int getConnectStatus();
    int getEnableStatus();
    int getMassageStatus();

public:
    bool MoveL(int peroid, float point_start[6], float point_end[6], float vmax, float amax, float jmax, std::vector<std::vector<float> > &trajectory_point, vector<vector<float> > &trajectory_inf);
    float TrajectoryTime(float Q, float vmax, float amax, float jmax, VectorXf& para);
    int CorrentionParameters(VectorXf& para, int peroid);
    void S_position_velocity(float t, VectorXf para, float q_dq[3]);
    void Equation_Root5(double coffe[6], double Roots_result[5][2]);
    void Equation_Root4(double coffe[5], double Roots_result[4][2]);
    void Equation_Root3(double coffe[4], double Roots_result[3]);
    void Equation_Root2(double coffe[3], double Roots_result[2], double Detla);

public:
    USBCAN *usbcan;
    TcpServer *m_tcpserver;
    XArmAPI *m_arm;
    pthread_t m_control;
public:
    //robot status
    int m_connectstate;        //机器人连接状态
    int m_enablestate;         //机器人使能标志
    int m_massagestate;        //按摩标志
    bool m_getpointstate;       //从文件读取按摩点位置
    bool m_usbcanstate;
    //socket instruction
    int m_connectrobot;
    int m_disconnectrobot;
    int m_enablerobot;
    int m_disablerobot;
    int m_massage;
    int m_resetrobot;
public:
    float m_posture[3];
    std::vector<std::vector<float> > m_trajectorypoint;//trajecory results
    vector<vector<float> > m_acupointposition;
    float m_desiredpose;//期望位置
    float m_desiredvelocity;//期望速度
    float m_desiredacceleration;//期望加速度
    float m_currentpose;//当前位置
    float m_currentvelocity;//当前速度
    float m_currentacceleration;//当前加速度
    float m_sensormessage;//力传感器数据
    float m_deltapose;//位置误差
    float m_deltavelocity;//速度误差
    float m_deltaacceleration;//加速度误差
    float m_lastdeltapose;//位置误差
    float m_lastdeltavelocity;//速度误差
    float m_lastdeltaacceleration;//加速度误差
    float m_M;//惯性系数
    float m_B;//阻尼系数
    float m_K;//刚度系数
    float m_deltaT;//

signals:
    void setConnect(bool status);
    void setEnable(bool status);
    void setMassage(bool status);


private:
    TrajectoryType m_trajectorytype;
    bool m_isStop;

public slots:
    //
    void startThreadSlot();
    //socket槽函数
    void setRobotConnect();
    void setRobotDisconnect();
    void setRobotEnable();
    void setRobotDisable();
    void setRobotMassage(vector<vector<float> > acupoint);
    void setRobotReset();
signals:

};

static RobotControl& GetRobotControlInstance()
{
    static RobotControl RobotControlInstance;
    return RobotControlInstance;
}

#endif // ROBOTCONTROL_H
