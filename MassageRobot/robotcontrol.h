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
    virtual int CreatThread();

public:
    /**
     * @brief ConnectRobot
     *          连接设备
     */
    void ConnectRobot();
    /**
     * @brief DisconnectRobot
     *          与设备断开连接
     */
    void DisconnectRobot();
    bool Isconnected();
    bool Isenabled();
    /**
     * @brief EnableRobot
     *          机器人使能/关闭
     * @param state
     *          true：使能
     *          false：关闭
     */
    void EnableRobot();
    void DisableRobot();
    void ResetRobot();
    /**
     * @brief SetMassage
     *          按摩开启/关闭
     * @param state
     *          true：使能
     *          false：关闭
     */
    int SetMassage(vector<vector<float> > acupoint);

    void RobotMassage();

    void MassagePerform();

    void getForceData(float data);

    void setConnectStatus(bool status);
    void setEnableStatus(bool status);
    void setMassageStatus(bool status);
    int getConnectStatus();
    int getEnableStatus();
    int getMassageStatus();

public:
    static void *RobotControl_thread(void *arg);
    void SetTcpOffest(float tcp_offest[6]);
    void SetTcpOffsetFromFile(const std::string &filename);
    void SetBaseOffest(float base_offest[6]);
    void SetBaseOffsetFromFile(const std::string &filename);

    /**
     * @brief ProcessPosition
     *          将参考坐标系有相机坐标系变为基坐标系，添加姿态信息{180,0,0}
     * @param point_in_camera
     * @param run_point
     */
    void ProcessPosition(float point_in_camera[3], std::vector<float> &run_point);

    /**
     * @brief ReadPointFromTXT
     *          从txt文件中读取穴位位置，不包含姿态信息
     * @param filename
     * @param point_vector
     * @return
     */
    int ReadPointFromTXT(const std::string &filename, std::vector<std::vector<float> > &point_vector);

    /**
     * @brief MoveL
     *          直线轨迹规划
     * @param peroid
     * @param point_start
     * @param point_end
     * @param vmax
     * @param amax
     * @param jmax
     * @param trajectory_point
     * @return
     */
    bool MoveL(int peroid, float point_start[6], float point_end[6], float vmax, float amax, float jmax, std::vector<std::vector<float> > &trajectory_point, vector<vector<float> > &trajectory_inf);
    bool MoveL_old(int peroid, float point_start[6], float point_end[6], float vmax, float amax, float jmax, std::vector<std::vector<float> > &trajectory_point, vector<vector<float> > &trajectory_inf);
    void Recalculate_V_A(double Q, float Vmax, float Amax, float Jmax, double Tmax, float V_A[2]);
    void Trajectory_Four(double Q, float Tmax, float *jmax, float *amax, float *vmax);
    void Trajectory_Five(double Q, float Tmax, float *Jmax, float *Amax, float *Vmax, float AMAX, float VMAX);
    void Normalization_DoubleS(int T,float S, float vmax, float amax, float Jmax, float v_0, float v_1, double Tmax, float lambda[],VectorXf &para,int length);

    /**
     * @brief TrajectoryTime
     *          计算直线规划需要的时间
     * @param peroid
     * @param Q
     * @param vmax
     * @param amax
     * @param jmax
     * @param para
     * @return
     */
    float TrajectoryTime(float Q, float vmax, float amax, float jmax, VectorXf& para);

    /**
     * @brief CorrentionParameters
     *          时间周期化后更新速度，加速度，加加速度约束
     * @param para
     * @param peroid
     * @return
     */
    int CorrentionParameters(VectorXf& para, int peroid);

    /**
     * @brief S_position_velocity
     *          双S速度曲线规划结果，包括位移，速度和加速度信息
     * @param t
     * @param para
     * @param q_dq
     */
    void S_position_velocity(float t, VectorXf para, float q_dq[3]);

    /**
     * @brief Equation_Root5
     *          求解五次方程
     * @param coffe
     * @param Roots_result
     */
    void Equation_Root5(double coffe[6], double Roots_result[5][2]);

    /**
     * @brief Equation_Root4
     *          求解四次方程
     * @param coffe
     * @param Roots_result
     */
    void Equation_Root4(double coffe[5], double Roots_result[4][2]);

    /**
     * @brief Equation_Root3
     *          求解三次方程
     * @param coffe
     * @param Roots_result
     */
    void Equation_Root3(double coffe[4], double Roots_result[3]);

    /**
     * @brief Equation_Root2
     *          求解二次方程
     * @param coffe
     * @param Roots_result
     * @param Detla
     */
    void Equation_Root2(double coffe[3], double Roots_result[2], double Detla);

    void PoseToHomogenousMatrix4f(float pose[6], float matrix[4][4]);
    void IdentityMatrix(float matrix[4][4]);
    void InverseMatrix(float matrix[4][4], float inverse_matrix[4][4]);
    void TransposeMatrix(float matrix[3][3], float transpose_matrix[3][3]);
    void MatrixMultiplyVector4f(float matrix[4][4], float vector[4], float result[4]);
    void MatrixMultiplyVector3f(float matrix[3][3], float vector[3], float result[3]);

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
    float m_tcpoffset[6];               //
    float m_tcpoffsetmatrix[4][4];      //
    float m_baseoffset[6];              //相机坐标系在基坐标系下的表示，xyz+rpy
    float m_baseoffsetmatrix[4][4];     //相机坐标系在基坐标系下的表示，矩阵形式
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
    int number;

signals:
    void setConnect(bool status);
    void setEnable(bool status);
    void setMassage(bool status);


private:
    TrajectoryType m_trajectorytype;
    bool m_isStop;

public slots:
    void startThreadSlot();
    //socket槽函数
    void setRobotConnect();
    void setRobotDisconnect();
    void setRobotEnable();
    void setRobotDisable();
    void setRobotMassage(vector<vector<float> > acupoint);
    void setRobotReset();
//public:
//    static RobotControl& GetRobotControlInstance()
//    {
//        static RobotControl RobotControlInstance;
//        return RobotControlInstance;
//    }

signals:

};

static RobotControl& GetRobotControlInstance()
{
    static RobotControl RobotControlInstance;
    return RobotControlInstance;
}

#endif // ROBOTCONTROL_H
