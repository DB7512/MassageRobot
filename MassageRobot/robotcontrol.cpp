#include "robotcontrol.h"
#include <fstream>
#include <iostream>
#include <QDebug>
#include <QTimer>
#include <eigen-3.4.0/Eigen/Geometry>
#include <math.h>
#include <QTextStream>
#include <QFile>
using namespace std;

#define PI 3.14159265358979
RobotControl::RobotControl(QObject *parent)
    : QObject{parent}
{
    m_isStop = false;
    m_arm = NULL;
    m_getpointstate = false;
    m_usbcanstate = false;
    m_connectstate      = false;
    m_enablestate       = false;
    m_massagestate      = false;
    m_connectrobot      = false;
    m_disconnectrobot   = false;
    m_enablerobot       = false;
    m_disablerobot      = false;
    m_massage           = false;
    m_resetrobot        = false;
    m_arm = new XArmAPI("192.168.1.238", false, true, true, true, true, false, true, true, 0);
    m_M = 1;
    m_B = 20;
    m_K = 100;
    m_deltaT = 0.02;
    usbcan = NULL;
    usbcan = new USBCAN;
    connect(usbcan, &USBCAN::getSensorData, this, &RobotControl::getForceData, Qt::DirectConnection);
    m_tcpserver = new TcpServer;
    connect(m_tcpserver, &TcpServer::connection, this, &RobotControl::setRobotConnect, Qt::DirectConnection);
    connect(m_tcpserver, &TcpServer::disconnection, this, &RobotControl::setRobotDisconnect, Qt::DirectConnection);
    connect(m_tcpserver, &TcpServer::enableRobot, this, &RobotControl::setRobotEnable, Qt::DirectConnection);
    connect(m_tcpserver, &TcpServer::disableRobot, this, &RobotControl::setRobotDisable, Qt::DirectConnection);
    connect(m_tcpserver, &TcpServer::startMassage, this, &RobotControl::setRobotMassage, Qt::DirectConnection);
    connect(m_tcpserver, &TcpServer::resetRobot, this, &RobotControl::setRobotReset, Qt::DirectConnection);
    connect(this, &RobotControl::setConnect, m_tcpserver, &TcpServer::connectStatus, Qt::DirectConnection);
    connect(this, &RobotControl::setEnable, m_tcpserver, &TcpServer::enableStatus, Qt::DirectConnection);
    connect(this, &RobotControl::setMassage, m_tcpserver, &TcpServer::massageStatus, Qt::DirectConnection);
    setConnectStatus(false);
    setEnableStatus(false);
    setMassageStatus(false);
}


RobotControl::~RobotControl()
{
    m_isStop = true;
    setConnectStatus(false);
    setEnableStatus(false);
    setMassageStatus(false);
    if(m_arm) {
        delete m_arm;
        m_arm = NULL;
    }
}

void RobotControl::ConnectRobot()
{
    if(!m_connectstate) {           //当前未连接
        m_arm->connect();
        sleep_milliseconds(500);
        if(m_arm->error_code != 0) m_arm->clean_error();
        if(m_arm->warn_code != 0) m_arm->clean_warn();
        if(m_arm->is_connected()) {
            setConnectStatus(true);
            qDebug("设备连接成功。\n");
            if (m_arm->error_code != 0) m_arm->clean_error();
            if (m_arm->warn_code != 0) m_arm->clean_warn();
            m_arm->motion_enable(false);
            sleep_milliseconds(500);
            setEnableStatus(false);
        } else {
            qDebug("设备连接失败，请重新连接。\n");
        }
    } else {
        if(m_arm->error_code != 0) m_arm->clean_error();
        if(m_arm->warn_code != 0) m_arm->clean_warn();
        if(m_arm->is_connected()) {
            qDebug("设备已经连接。\n");
        } else {
            setConnectStatus(false);
            qDebug("设备连接异常，请重新连接。\n");
        }
    }
}

void RobotControl::DisconnectRobot()
{
    if(m_enablestate) {
        DisableRobot();//关闭机器人
    }
    m_arm->disconnect();//断开连接
    if(!m_arm->is_connected()) {
        setConnectStatus(false);
        qDebug("Disconnected successfully. \n");
    }else {
        qDebug("Disconnection failed.\n");
    }
}

bool RobotControl::Isconnected()
{
    if(m_arm->is_connected()) {
        return true;
    } else {
        return false;
    }
}

bool RobotControl::Isenabled()
{
    if(m_enablestate == true) {
        return true;
    }else {
        return false;
    }
}

void RobotControl::EnableRobot()
{
    if(m_arm->is_connected()) {
        if (m_arm->error_code != 0) m_arm->clean_error();
        if (m_arm->warn_code != 0) m_arm->clean_warn();
        if(!m_enablestate) {
            m_arm->motion_enable(true);
            int status = 0;
            int ret = m_arm->get_state(&status);
            qDebug()<<"ret"<<ret<<"status"<<status;
            qDebug("enable1");
            m_arm->set_mode(0);
            m_arm->set_state(0);
            sleep_milliseconds(500);
            setEnableStatus(true);
            qDebug("机器人使能成功。\n");
            if(m_connectstate == true && m_enablestate == true) {
                if(!m_usbcanstate) {
                    if(usbcan->openDevice(4, 0, 500)) {
                        sleep_milliseconds(50);
                        if(usbcan->initCAN()) {
                            sleep_milliseconds(50);
                            if(usbcan->startCAN()) {
                                sleep_milliseconds(50);
                                usbcan->initSensor();
                                sleep_milliseconds(50);
                                usbcan->start();
                                m_usbcanstate = true;
                            }
                        }
                    }else {
                        qDebug("CAN打开失败。\n");
                    }
                }
            }
        }else {
            qDebug("已经使能。");
        }
    } else {
        setConnectStatus(false);
        setEnableStatus(false);
        qDebug("设备未连接，请重新连接。\n");
    }

}

void RobotControl::DisableRobot()
{
    if(m_arm->is_connected()) {
        if(m_enablestate) {
            if (m_arm->error_code != 0) m_arm->clean_error();
            if (m_arm->warn_code != 0) m_arm->clean_warn();
            m_arm->motion_enable(false);
            sleep_milliseconds(500);
            setEnableStatus(false);
            if(m_usbcanstate) {
                usbcan->closeDevice();
                m_usbcanstate = false;
                qDebug("机器人关闭成功。\n");
            }else {
                qDebug("CAN异常。\n");
            }
        }else {
            qDebug("Already disabled.\n");
        }
    } else {
        setConnectStatus(false);
        setEnableStatus(false);
        qDebug("设备未连接，无法关闭。\n");
    }
}

void RobotControl::ResetRobot()
{
    if(m_arm->is_connected()) {
        if (m_arm->error_code != 0) m_arm->clean_error();
        if (m_arm->warn_code != 0) m_arm->clean_warn();
        m_arm->set_mode(0);
        m_arm->set_state(0);
        sleep_milliseconds(500);
        m_arm->reset(true);
        qDebug("Reset successfully.\n");
    } else {
        qDebug("Disconnected, cannot reset.\n");
        return;
    }
}

void RobotControl::MassagePerform()
{
    setMassageStatus(true);
    int acupoint_number = 0;//穴位点数量
    int run_point_number = 0;//运行点数量
    vector<float> final_point(6);//最终点
    float point[3] = {0.0};
    float ftemp_point[6] = {0.0};
    int temp_number = 0;
    if (m_arm->error_code != 0) m_arm->clean_error();
    if (m_arm->warn_code != 0) m_arm->clean_warn();
    float start_point[6] = {0.0};
    float end_point[6] = {0.0};
    vector<vector<float> >interpolation_result;//插值点
    vector<vector<float> >interpolation_inf;
    vector<float> interpolation_point(6);
    vector<float> interpolation_pointinf(3);
    int interpolation_number = 0;
    float pose[6] = {0.0};
    float offset = 20.0;
    while(m_massagestate) {
        vector<vector<float> > run_point;//运行点
        vector<float> temp_point;//临时点
        run_point.clear();
        temp_point.clear();
        //读取穴位点后，补全对应穴位点的进入点和离开点，包括位置和姿态
        acupoint_number = m_acupointposition.size();//获取穴位点数量
        for(int i = 0; i < acupoint_number*3; i++) {
            temp_number = i/3;
            temp_point = m_acupointposition.at(temp_number);
            for(int j = 0; j < 3; j++) {
                point[j] = temp_point.at(j);
            }
            temp_point.clear();
            if(i%3 == 0 || i%3 == 2) {
                point[0] -= 150;
                for(int j = 0; j < 3; j++) {
                    final_point[j] = point[j];
                }
                final_point[3] = 180.0;
                final_point[4] = 0.0;
                final_point[5] = 0.0;
                run_point.push_back(final_point);
            } else {
                for(int j = 0; j < 3; j++) {
                    final_point[j] = point[j];
                }
                final_point[3] = 180.0;
                final_point[4] = 0.0;
                final_point[5] = 0.0;
                run_point.push_back(final_point);
            }
        }
        run_point_number = run_point.size();//计算路点个数
        for(int i = 0; i < run_point_number; i++) {
            temp_point = run_point.at(i);
            temp_number = temp_point.size();
            for(int j = 0; j < temp_number; j++) {
                ftemp_point[j] = temp_point.at(j);
                //
                if(i%3 == 0) {
                    start_point[j] = ftemp_point[j];
                } else if(i%3 == 1) {
                    end_point[j] = ftemp_point[j];
                }
            }
            if(i%3 == 1) {
                //move to target point with (x - 0.005m)
                float trg_point[6] = {0.0};
                for (int var = 0; var < 6; ++var) {
                    trg_point[var] = end_point[var];
                }
                m_arm->set_mode(0);
                m_arm->set_state(0);
                sleep_milliseconds(500);
                ftemp_point[0] = trg_point[0] - offset;
                m_arm->set_position(ftemp_point,true);
                sleep_milliseconds(100);
                for(int m = 0; m < 6; m++) {
                    if(m == 0) {
                        start_point[m] = trg_point[m] - offset;
                        end_point[m] = trg_point[m] + 5.0;
                    } else {
                        start_point[m] = trg_point[m];
                        end_point[m] = trg_point[m];
                    }
                }
                m_currentpose = start_point[0]/1000.0;
                MoveL(50, start_point, end_point, 0.1, 3.0, 6.0, interpolation_result, interpolation_inf);
                interpolation_number = interpolation_result.size();//插值个数
                m_arm->set_mode(1);
                m_arm->set_state(0);
                sleep_milliseconds(500);
                m_lastdeltapose = 0.0;
                m_lastdeltavelocity = 0.0;
                m_lastdeltaacceleration = 0.0;
                for(int j = 0; j < 500; j++) {
                    if(j < interpolation_number - 1) {
                        m_M = 1.0;
                        m_B = 20.0;
                        m_K = 100.0;
                        interpolation_point = interpolation_result.at(j);
                        interpolation_pointinf = interpolation_inf.at(j);
                        for (int var = 0; var < 6; ++var) {
                            pose[var] = interpolation_point[var];
                        }
                        m_desiredpose = pose[0]/1000.0;
                        m_desiredvelocity = interpolation_pointinf[1];
                        m_desiredacceleration = interpolation_pointinf[2];
                        //导纳控制
                        m_lastdeltapose = m_currentpose - m_desiredpose - (-m_desiredvelocity)*m_deltaT;
                        m_lastdeltavelocity = m_currentvelocity - m_desiredvelocity - (-m_desiredacceleration)*m_deltaT;
                        m_lastdeltaacceleration = m_currentacceleration - m_desiredacceleration;
                        qDebug()<<"force"<<m_sensormessage;
                        if(m_sensormessage < 8.0) {
                            m_sensormessage = 0.0;
                        } else {
                            m_sensormessage = m_sensormessage - 8.0;
                        }

                        m_deltaacceleration = 1.0/m_M*(-m_sensormessage
                                                     - m_B*(m_lastdeltavelocity) - m_K*(m_lastdeltapose));
                        m_deltavelocity = m_lastdeltavelocity - m_desiredvelocity + m_deltaT*m_deltaacceleration;
                        m_deltapose = m_lastdeltapose + m_deltavelocity*m_deltaT;
                        if(m_deltapose > 100) {
                            int amamam = 0;
                        }
                        m_currentpose = m_deltapose + m_desiredpose;
                        m_currentvelocity = m_deltavelocity + m_desiredvelocity;
                        m_currentacceleration = m_deltaacceleration + m_desiredacceleration;
                        pose[0] = (m_deltapose + m_desiredpose)*1000;
                    } else {
                        //系数
                        m_M = 0.8;
                        m_B = 30.0;
                        m_K = 60.0;
                        //期望位置
                        m_desiredpose = (trg_point[0] + 5.0)/1000.0;
                        m_desiredvelocity = 0.0;
                        m_desiredacceleration = 0.0;
                        //前一时刻误差
                        m_lastdeltapose = m_currentpose - m_desiredpose;
                        m_lastdeltavelocity = m_currentvelocity - m_desiredvelocity;
                        m_lastdeltaacceleration = m_currentacceleration - m_desiredacceleration;
                        //导纳控制
                        qDebug()<<"force"<<m_sensormessage;
                        if(m_sensormessage < 8.0) {
                            m_sensormessage = 0.0;
                        } else {
                            m_sensormessage = m_sensormessage - 8.0;
                        }
                        m_deltaacceleration = 1.0/m_M*(-m_sensormessage- m_B*(m_lastdeltavelocity) - m_K*(m_lastdeltapose));
                        m_deltavelocity = m_lastdeltavelocity - m_desiredvelocity + m_deltaT*m_deltaacceleration;
                        m_deltapose = m_lastdeltapose + m_deltavelocity*m_deltaT;
                        //更新实际位置
                        m_currentpose = m_deltapose + m_desiredpose;
                        m_currentvelocity = m_deltavelocity + m_desiredvelocity;
                        m_desiredacceleration = m_deltaacceleration + m_desiredacceleration;
                        //下发
                        pose[0] = (m_deltapose + m_desiredpose)*1000;
                    }
                    qDebug()<<"force"<<m_sensormessage<<"dx"<<m_deltapose<<"x"<<pose[0]<<"m_desiredpose"<<m_desiredpose;
                    m_arm->set_servo_cartesian(pose);
                    sleep_milliseconds(20);
                }
                vector<vector<float>> ().swap(interpolation_result);
                vector<vector<float>> ().swap(interpolation_inf);

            } else {
                m_arm->set_mode(0);
                m_arm->set_state(0);
                sleep_milliseconds(500);
                m_arm->set_position(ftemp_point,true);
                sleep_milliseconds(100);
            }
        }
        float zero_point[6] = {207.0,0.0,112.0,180.0,0.0,0.0};
        m_arm->set_mode(0);
        m_arm->set_state(0);
        sleep_milliseconds(500);
        m_arm->set_position(zero_point,true);
        sleep_milliseconds(100);
        setMassageStatus(false);
        m_getpointstate = false;
        vector<vector<float>> ().swap(run_point);
        vector<float> ().swap(temp_point);
    }
}

void RobotControl::getForceData(float data)
{
    //    qDebug("data");
    m_sensormessage = data;
}

void RobotControl::setConnectStatus(bool status)
{
    m_connectstate = status;
    emit setConnect(status);
    //    qDebug()<<"status"<<m_connectstate;
}

void RobotControl::setEnableStatus(bool status)
{
    m_enablestate = status;
    emit setEnable(status);
}

void RobotControl::setMassageStatus(bool status)
{
    m_massagestate = status;
    emit setMassage(status);
}

int RobotControl::getConnectStatus()
{
    return m_connectstate;
}

int RobotControl::getEnableStatus()
{
    return m_enablestate;
}

int RobotControl::getMassageStatus()
{
    return m_massagestate;
}

bool RobotControl::MoveL(int peroid, float point_start[6], float point_end[6], float vmax, float amax, float jmax,
std::vector<std::vector<float> > &trajectory_point, vector<vector<float> > &trajectory_inf)
{
    float Q = 0.0;//位移
    float trajectory_ftime = 0.0;
    int interpolation_peroid_num = 0;
    float t = 0.0;//插值时刻
    float q_dq[3] = {0.0};//插值结果：位移，速度，加速度
    float detla_x = point_end[0] - point_start[0];//x轴位移
    float detla_y = point_end[1] - point_start[1];//y轴位移
    float detla_z = point_end[2] - point_start[2];//z轴位移
    VectorXf para = VectorXf::Zero(14);;//速度规划参数
    //计算位移
    Q = sqrt(pow(detla_x,2) + pow(detla_y,2) + pow(detla_z,2))/1000.0;
    //计算规划时间
    para(9) = vmax;
    para(10) = amax;
    para(13) = jmax;
    trajectory_ftime = TrajectoryTime(Q, vmax, amax, jmax, para);
    CorrentionParameters(para, peroid);
    vmax = para(9);
    amax = para(10);
    jmax = para(13);
    trajectory_ftime = TrajectoryTime(Q, vmax, amax, jmax, para);
    double temp = trajectory_ftime/(1.0/peroid);
    interpolation_peroid_num = round(temp);
    if(trajectory_ftime < 1e-6) return false;
    float lambda[interpolation_peroid_num];//归一化参数
    vector<float> interpolation_point(6);//插值结果
    vector<float> infpoint(3);
    for (int i = 0; i < interpolation_peroid_num; i++) {
        //容器初始化
        for(int j = 0; j < 6; j++) {
            if(j < 3) {
                interpolation_point[j] = 0.0;
                infpoint[j] = 0.0;
            } else {
                interpolation_point[j] = point_start[j];
            }
        }
        lambda[i] = 0.0;
        t = i*(1.0/peroid);
        //计算轨迹
        S_position_velocity(t, para, q_dq);
        for (int var = 0; var < 3; ++var) {
            infpoint[var] = q_dq[var];
        }
        if(Q!=0)
            lambda[i] = q_dq[0] / Q;
        interpolation_point[0] = point_start[0] + detla_x * lambda[i];
        interpolation_point[1] = point_start[1] + detla_y * lambda[i];
        interpolation_point[2] = point_start[2] + detla_z * lambda[i];
        trajectory_point.push_back(interpolation_point);
        trajectory_inf.push_back(infpoint);
    }
    return true;
}

float RobotControl::TrajectoryTime( float Q, float vmax, float amax, float jmax, VectorXf& para)
{
    float T = 0.0;
    float v_0 = 0.0;
    float v_1 = 0.0;
    float v_max = vmax;
    float a_max = amax;
    float j_max = jmax;
    float Tj1=0, Tj2=0, Ta=0, Td=0, Tv=0;
    float a_lima=0, a_limd=0;
    float vlim=0;
    if ((v_max - v_0) * j_max < pow(a_max, 2)) {
        Tj1 = sqrt((v_max - v_0) / j_max);
        Ta = 2 * Tj1;
        a_lima = j_max * Tj1;
    } else {
        Tj1 = a_max / j_max;
        Ta = Tj1 + (v_max - v_0) / a_max;
        a_lima = a_max;
    }
    if ((v_max - v_1) * j_max < pow(a_max, 2)) {
        Tj2 = sqrt((v_max - v_1) / j_max);
        Td = 2 * Tj2;
        a_limd = -j_max * Tj2;
    } else {
        Tj2 = a_max / j_max;
        Td = Tj2 + (v_max - v_1) / a_max;
        a_limd = -a_max;
    }
    Tv = (Q) / v_max - (Ta / 2) * (1 + v_0 / v_max) - (Td / 2) * (1 + v_1 / v_max);
    if (Tv > 0) {
        vlim = v_max;
        T = Ta + Tv + Td;
    } else {
        Tv = 0;
        float Tj=0;
        float delta=0;
        Tj = a_max / j_max;
        Tj1 = Tj;
        Tj2 = Tj;
        delta = (pow(a_max, 4) / pow(j_max, 2)) + 2 * (pow(v_0, 2) + pow(v_1, 2)) + a_max * (4 * Q - 2 * (a_max / j_max) * (v_0 + v_1));
        Ta = ((pow(a_max, 2) / j_max) - 2.0 * v_0 + sqrt(delta)) / (2.0 * a_max);
        Td = ((pow(a_max, 2) / j_max) - 2.0 * v_1 + sqrt(delta)) / (2.0 * a_max);
        if (Ta < 0 || Td < 0) {
            if (Ta < 0) {
                Ta = 0;
                Tj1 = 0;
                Td = 2 * Q / (v_0 + v_1);
                Tj2 = (j_max * Q - sqrt(j_max * (j_max * pow(Q, 2) + pow(v_1 + v_0, 2) * (v_1 - v_0)))) / (j_max * (v_1 + v_0));
                a_lima = 0;
                a_limd = -j_max * Tj2;
                vlim = v_0;
                T = Ta + Tv + Td;
            } else if (Td < 0) {
                Td = 0;
                Tj2 = 0;
                Ta = 2 * Q / (v_0 + v_1);
                Tj1 = (j_max * Q - sqrt(j_max * (j_max * pow(Q, 2)) - pow(v_1 + v_0, 2) * (v_1 - v_0))) / (j_max * (v_1 + v_0));
                a_lima = j_max * Tj1;
                a_limd = 0;
                vlim = v_0 + a_lima * (Ta - Tj1);
                T = Ta + Tv + Td;
            }
        } else if (Ta >= 2 * Tj && Td >= 2 * Tj) {
            a_lima = a_max;
            a_limd = -a_max;
            vlim = v_0 + a_lima * (Ta - Tj);
            T = Ta + Tv + Td;
        } else if (Ta < 2 * Tj1 && Td >= 2 * Tj2) {
            double coeff[6] = { 0 };
            double x[5][2] = { {0} };
            coeff[0] = 1;
            coeff[1] = 2 * a_max;
            coeff[2] = 2 * j_max * v_0 + pow(a_max, 2);
            coeff[3] = 4 * j_max * a_max * v_0;
            coeff[4] = j_max * ((pow(v_0, 2) - pow(v_1, 2)) * j_max + pow(a_max, 2) * (v_0 + v_1) - 2 * j_max * a_max * Q);
            coeff[5] = 0;
            Equation_Root5(coeff, x);
            a_lima = 0;
            for (int j = 0; j < 5; j++) {
                if (x[j][0]<0 || x[j][0]>a_max) {
                    x[j][0] = 0;
                }
                if (x[j][0] > a_lima) {
                    a_lima = x[j][0];
                }
            }
            Tj1 = a_lima / j_max;
            Tj2 = a_max / j_max;
            Ta = 2 * Tj1;
            vlim = Tj1 * a_lima + v_0;
            Td = (vlim - v_1) / a_max + Tj2;
            T = Ta + Tv + Td;
        } else if (Ta >= 2 * Tj1 && Td < 2 * Tj2) {
            double coeff[6] = { 0 };
            double x[5][2] = { {0} };
            coeff[0] = 1;
            coeff[1] = 2 * a_max;
            coeff[2] = 2 * j_max * v_1 + pow(a_max, 2);
            coeff[3] = 4 * j_max * a_max * v_1;
            coeff[4] = j_max * ((pow(v_1, 2) - pow(v_0, 2)) * j_max + pow(a_max, 2) * (v_0 + v_1) - 2 * j_max * a_max * Q);
            coeff[5] = 0;
            Equation_Root5(coeff, x);
            a_limd = 0;
            for (int j = 0; j < 5; j++) {
                if (x[j][0]<0 || x[j][0]>a_max) {
                    x[j][0] = 0;
                }
                if (x[j][0] > a_limd) {
                    a_limd = x[j][0];
                }
            }
            Tj1 = a_max / j_max;
            Tj2 = a_limd / j_max;
            Td = 2 * Tj2;
            vlim = Tj2 * a_limd + v_1;
            Ta = (vlim - v_0) / a_max + Tj1;
            T = Ta + Tv + Td;
        } else if (Ta < 2 * Tj1 && Td < 2 * Tj2) {
            if (fabs(v_0 - v_1) > 0) {
                double coeff[5] = { 0 };
                double x[4][2] = { {0} };
                coeff[0] = j_max * (v_0 - v_1);
                coeff[1] = -2 * pow(j_max, 2) * Q;
                coeff[2] = pow(j_max, 2) * pow(v_0 - v_1, 2);
                coeff[3] = -4 * pow(j_max, 3) * v_0 * Q;
                coeff[4] = pow(j_max, 4) * pow(Q, 2) - pow(j_max, 3) * (v_0 - v_1) * pow(v_0 + v_1, 2);
                Equation_Root4(coeff, x);
                a_lima = 0;
                a_limd = 0;
                for (int j = 0; j < 4; j++) {
                    if (x[j][0]<0 || x[j][0]>a_max) {
                        x[j][0] = 0;
                    }
                    if (x[j][0] > a_lima) {
                        a_lima = x[j][0];
                    }
                }
                a_limd = sqrt(pow(a_lima, 2) + j_max * (v_0 - v_1));
                Tj1 = a_lima / j_max;
                Tj2 = a_limd / j_max;
                Ta = 2 * Tj1;
                Td = 2 * Tj2;
                vlim = Tj2 * a_limd + v_1;
                T = Ta + Tv + Td;
            } else {
                double coeff[4] = { 0 };
                double x[3] = { 0 };
                coeff[0] = 2 * pow(j_max, 2) * Q;
                coeff[1] = 0;
                coeff[2] = 4 * pow(j_max, 3) * v_0 * Q;
                coeff[3] = -pow(j_max, 4) * pow(Q, 2);
                Equation_Root3(coeff, x);
                a_lima = 0;
                a_limd = 0;
                for (int j = 0; j < 3; j++) {
                    if (x[j]<0 || x[j]>a_max) {
                        x[j] = 0;
                    }
                    if (x[j] > a_lima) {
                        a_lima = x[j];
                    }
                }
                a_limd = sqrt(pow(a_lima, 2) + j_max * (v_0 - v_1));
                Tj1 = a_lima / j_max;
                Tj2 = a_limd / j_max;
                Ta = 2 * Tj1;
                Td = 2 * Tj2;
                vlim = Tj2 * a_limd + v_1;
                T = Ta + Tv + Td;
            }
        }
    }
    para << Ta, Tv, Td, Tj1, Tj2, 0, Q, v_0, v_1, vlim, a_max, a_lima, a_limd, j_max;
    if(Tv > 0) {
        if(Ta > 2*Tj1) {
            m_trajectorytype = TraUniTra;
        }else {
            m_trajectorytype = TriUniTri;
        }
    }else {
        if(Ta > 2*Tj1) {
            m_trajectorytype = TraTra;
        }else {
            m_trajectorytype = TriTri;
        }
    }
    return T;
}

int RobotControl::CorrentionParameters(VectorXf &para, int peroid)
{
    float Ta, Tv, Td, Tj1, Tj2, Q, v_0, v_1, vlim, a_max, a_lima, a_limd, j_max;
    Ta      = para(0);
    Tv      = para(1);
    Td      = para(2);
    Tj1     = para(3);
    Tj2     = para(4);
    Q       = para(6);
    v_0     = para(7);
    v_1     = para(8);
    vlim    = para(9);
    a_max   = para(10);
    a_lima  = para(11);
    a_limd  = para(12);
    j_max   = para(13);
    float Te = ceil((Ta+Tv+Td)/(1.0/peroid)) * (1.0/peroid) - (Ta+Tv+Td);
    float Tj1_new = 0.0, Ta_new = 0.0, Jmax_new = 0.0;
    if(m_trajectorytype == TraUniTra || m_trajectorytype == TriUniTri) {
        Tj1_new = Tj1 + Te*0.5;
        Ta_new = Ta + Te;
        Jmax_new = vlim/((Ta_new - Tj1_new)*Tj1_new);
        a_max = Tj1_new*Jmax_new;
        vlim = a_max*(Ta_new - Tj1_new);
    }else if(m_trajectorytype == TraTra || m_trajectorytype == TriTri) {
        Tj1_new = Tj1 + Te*0.25;
        Ta_new = Ta + Te*0.5;
        Jmax_new = vlim/((Ta_new - Tj1_new)*Tj1_new);
        a_max = Tj1_new*Jmax_new;
        vlim = a_max*(Ta_new - Tj1_new);
    }
    para(13) = Jmax_new;
    para(10) = a_max;
    para(9)  = vlim;
}

void RobotControl::S_position_velocity(float t, VectorXf para, float q_dq[3])
{
    float Ta, Tv, Td, Tj1, Tj2, p1, p2, v_0, v_1, vlim, a_max, a_lima, a_limd, j_max;
    Ta = para(0);
    Tv = para(1);
    Td = para(2);
    Tj1 = para(3);
    Tj2 = para(4);
    p1 = para(5);
    p2 = para(6);
    v_0 = para(7);
    v_1 = para(8);
    vlim = para(9);
    a_max = para(10);
    a_lima = para(11);
    a_limd = para(12);
    j_max = para(13);
    float T;
    float q = 0, dq = 0, ddq = 0;
    T = Ta + Tv + Td;
    if (t >= 0 && t < Tj1) {
        q   =   v_0 * t + j_max * pow(t, 3) / 6;
        dq  =   v_0 + j_max * (pow(t, 2) * 0.5);
        ddq =   j_max * t;
    } else if (t >= Tj1 && t < Ta - Tj1) {
        q   =   v_0 * t + (a_lima / 6) * (3 * pow(t, 2) - 3 * Tj1 * t + pow(Tj1, 2));
        dq  =   v_0 + a_lima * (t - Tj1 * 0.5);
        ddq =   a_lima;
    } else if (t >= Ta - Tj1 && t < Ta) {
        q   =   (vlim + v_0) * (Ta / 2) - vlim * (Ta - t) + j_max * (pow(Ta - t, 3) / 6);
        dq  =   vlim - j_max * (pow(Ta - t, 2) * 0.5);
        ddq =   - j_max * (t -Ta);
    } else if (t >= Ta && t < Ta + Tv) {
        q   =   (vlim + v_0) * (Ta * 0.5) + vlim * (t - Ta);
        dq  =   vlim;
        ddq =   0;
    } else if (t >= Ta + Tv && t < T - Td + Tj2) {
        q   =   fabs(p2 - p1) - (vlim + v_1) * (Td * 0.5) + vlim * (t - T + Td) - j_max * (pow(t - T + Td, 3) / 6);
        dq  =   vlim - j_max * (pow(t - T + Td, 2) * 0.5);
        ddq =   - j_max * (t - T + Td);
    } else if (t >= T - Td + Tj2 && t < T - Tj2) {
        q   =   fabs(p2 - p1) - (vlim + v_1) * (Td * 0.5) + vlim * (t - T + Td) + (a_limd / 6) * (3 * pow(t - T + Td, 2) - 3 * Tj2 * (t - T + Td) + pow(Tj2, 2));
        dq  =   vlim + a_limd * (t - T + Td - Tj2 * 0.5);
        ddq =   a_limd;
    } else {
        q   =   fabs(p2 - p1) - v_1 * (T - t) - j_max * (pow(T - t, 3) / 6);
        dq  =   v_1 + j_max * (pow(t - T, 2) * 0.5);
        ddq =   j_max * (t - T);
    }
    q_dq[0] = q;
    q_dq[1] = dq;
    q_dq[2] = ddq;
}

void RobotControl::Equation_Root5(double coffe[6], double Roots_result[5][2])
{
    Eigen::Matrix<double, 5, 5>matrix_coffe;
    Eigen::Matrix<complex<double>, Eigen::Dynamic, Eigen::Dynamic>matrix_sigenvalues;
    Eigen::VectorXd rl = VectorXd(5);
    Eigen::VectorXd ig = VectorXd(5);
    matrix_coffe << -coffe[1] / coffe[0], -coffe[2] / coffe[0], -coffe[3] / coffe[0], -coffe[4] / coffe[0], -coffe[5] / coffe[0],
            1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0;

    matrix_sigenvalues = matrix_coffe.eigenvalues();
    rl << matrix_sigenvalues.real();
    ig << matrix_sigenvalues.imag();

    for (int i = 0; i < 5; i++)
    {
        if (ig(i) < 0.000001 && ig(i) > -0.000001) {
            Roots_result[i][0] = rl(i);
            Roots_result[i][1] = 0;
        }
        else {
            Roots_result[i][0] = 0;
            Roots_result[i][1] = 0;
        }
    }
}

void RobotControl::Equation_Root4(double coffe[5], double Roots_result[4][2])
{
    Matrix<double, 5, 5>matrix_coffe;
    Matrix<complex<double>, Dynamic, Dynamic>matrix_sigenvalues;
    VectorXd rl = VectorXd(5);
    VectorXd ig = VectorXd(5);
    matrix_coffe << -coffe[1] / coffe[0], -coffe[2] / coffe[0], -coffe[3] / coffe[0], -coffe[4] / coffe[0], -coffe[5] / coffe[0],
            1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0;

    matrix_sigenvalues = matrix_coffe.eigenvalues();
    rl << matrix_sigenvalues.real();
    ig << matrix_sigenvalues.imag();

    for (int i = 0; i < 5; i++)
    {
        if (ig(i) < 0.000001 && ig(i) > -0.000001) {
            Roots_result[i][0] = rl(i);
            Roots_result[i][1] = 0;
        }
        else {
            Roots_result[i][0] = 0;
            Roots_result[i][1] = 0;
        }
    }
}

void RobotControl::Equation_Root3(double coffe[4], double Roots_result[3])
{
    double a, b, c, d;
    double A, B, C;
    double delta;

    a = coffe[0];
    b = coffe[1];
    c = coffe[2];
    d = coffe[3];
    A = pow(b, 2) - 3 * a * c;
    B = b * c - 9 * a * d;
    C = pow(c, 2) - 3 * b * d;
    delta = pow(B, 2) - 4 * A * C;
    if (fabs(A) < 1e-6 && fabs(B) < 1e-6) {
        Roots_result[0] = -c / b;
        Roots_result[1] = Roots_result[0];
        Roots_result[2] = Roots_result[0];
        return;
    }
    if (delta > 0) {
        double Y1, Y2;
        double unreal;
        double Y_1, Y_2;
        int zero_1 = 0, zero_2 = 0;
        Y1 = A * b + 3 * a * (-B + sqrt(delta)) / 2;
        Y2 = A * b + 3 * a * (-B - sqrt(delta)) / 2;
        if (Y1 < 0) {
            Y1 = -Y1;
            zero_1 = 1;
        }
        if (Y2 < 0) {
            Y2 = -Y2;
            zero_2 = 1;
        }
        Y_1 = pow(Y1, 1.0 / 3);
        Y_2 = pow(Y2, 1.0 / 3);
        if (zero_1) {
            Y_1 = -Y_1;
        }
        if (zero_2) {
            Y_2 = -Y_2;
        }
        Roots_result[0] = (-b - (Y_1 + Y_2)) / (3 * a);
        unreal = (sqrt(3) / 2 * (Y_1 - Y_2)) / (3 * a);
        if (fabs(unreal) < 1e-6) {
            Roots_result[1] = (-b + 0.5 * (Y_1 + Y_2)) / (3 * a);
            Roots_result[2] = Roots_result[1];
        } else {
            Roots_result[1] = 0;
            Roots_result[2] = 0;
        }
        return;
    }
    if (fabs(delta) < 1e-6 && fabs(A) > 1e-6) {
        double K = B / A;
        Roots_result[0] = -b / a + K;
        Roots_result[1] = -0.5 * K;
        Roots_result[2] = Roots_result[1];
        return;
    }
    if (delta < 0 && A > 0) {
        double T, theta;
        T = (2 * A * b - 3 * a * B) / (2 * sqrt(pow(A, 3)));
        theta = acos(T);
        Roots_result[0] = (-b - 2 * sqrt(A) * cos(theta / 3)) / (3 * a);
        Roots_result[1] = (-b + sqrt(A) * (cos(theta / 3) + sqrt(3) * sin(theta / 3))) / (3 * a);
        Roots_result[2] = (-b + sqrt(A) * (cos(theta / 3) - sqrt(3) * sin(theta / 3))) / (3 * a);
        return;
    }
}

void RobotControl::Equation_Root2(double coffe[], double Roots_result[], double Detla)
{
    double D = sqrt(Detla);
    double A = 2 * coffe[0];
    Roots_result[0] = (-coffe[1] + D) / A;
    Roots_result[1] = (-coffe[1] - D) / A;
}

void RobotControl::startThreadSlot()
{
    while(!m_isStop) {
        if(m_connectrobot && !m_connectstate) {
            ConnectRobot();
            if(m_arm->is_connected() && m_connectstate) {
                m_connectrobot = false;
            }else {
                qDebug("connect error!");
            }
        }else if(m_disconnectrobot && m_connectstate) {
            DisconnectRobot();
            if(!m_arm->is_connected() && !m_connectstate) {
                m_disconnectrobot = false;
            }else {
                qDebug("disconnect error!");
            }
        }else if(m_connectstate) {
            if(m_arm->is_connected()) {
                //使能机器人
                if(m_enablerobot) {
                    EnableRobot();
                    if(m_enablestate) {
                        m_enablerobot = false;
                        qDebug("enable success");
                    }else {
                        qDebug("enable error!");
                    }
                }else if(m_enablestate) {
                    if(m_disablerobot) {
                        DisableRobot();
                        if(!m_enablestate) {
                            m_disablerobot = false;
                            qDebug("disable success");
                        }else {
                            qDebug("disenable error!");
                        }
                    }else if(m_resetrobot) {
                        if(m_enablestate && m_resetrobot) {//复位机器人
                            m_resetrobot = false;
                            ResetRobot();
                            qDebug("reset success.\n");
                        }
                    }else if(m_massage && !m_massagestate) {
                        m_massage = false;
                        MassagePerform();
                    }
                }
            }
        }
    }
}

void RobotControl::setRobotConnect()
{
    m_connectrobot = true;
}

void RobotControl::setRobotDisconnect()
{
    m_disconnectrobot = true;
}

void RobotControl::setRobotEnable()
{
    m_enablerobot = true;
}

void RobotControl::setRobotDisable()
{
    m_disablerobot = true;
}

void RobotControl::setRobotMassage(vector<vector<float> > acupoint)
{
    m_massage = true;
    m_acupointposition = acupoint;
}

void RobotControl::setRobotReset()
{
    m_resetrobot = true;
}
