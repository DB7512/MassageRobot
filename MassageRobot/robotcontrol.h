#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <string>
#include <vector>
#include "xarm/wrapper/xarm_api.h"
#include <thread>
class RobotControl
{
public:
    RobotControl();
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
    /**
     * @brief EnableRobot
     *          机器人使能/关闭
     * @param state
     *          true：使能
     *          false：关闭
     */
    void EnableRobot(bool state);
    /**
     * @brief SetMassage
     *          按摩开启/关闭
     * @param state
     *          true：使能
     *          false：关闭
     */
    void SetMassage(bool state);

private:
    static void *RobotControl_thread(void *arg);
    void SetTcpOffest(float tcp_offest[6]);
    void SetTcpOffsetFromFile(const std::string &filename);
    void SetBaseOffest(float base_offest[6]);
    void SetBaseOffsetFromFile(const std::string &filename);
    void SetPosition(float point_in_camera[3]);
    int ReadPointFromTXT(const std::string &filename, std::vector<std::vector<float> > &point_vector);

    void PoseToHomogenousMatrix4f(float pose[6], float matrix[4][4]);
    void IdentityMatrix(float matrix[4][4]);
    void InverseMatrix(float matrix[4][4], float inverse_matrix[4][4]);
    void TransposeMatrix(float matrix[3][3], float transpose_matrix[3][3]);
    void MatrixMultiplyVector4f(float matrix[4][4], float vector[4], float result[4]);
    void MatrixMultiplyVector3f(float matrix[3][3], float vector[3], float result[3]);

public:

    XArmAPI *m_arm;
    pthread_t m_control;
    bool m_connectstate;        //tcp连接状态
    bool m_enablestate;         //机器人使能标志
    bool m_messagestate;        //按摩标志
    bool m_getpointstate;       //从文件读取按摩点位置

    float m_tcpoffset[6];               //
    float m_tcpoffsetmatrix[4][4];      //
    float m_baseoffset[6];              //相机坐标系在基坐标系下的表示，xyz+rpy
    float m_baseoffsetmatrix[4][4];     //相机坐标系在基坐标系下的表示，矩阵形式
    float m_posture[3];
};

static RobotControl* GetRobotControlInstance()
{
    static RobotControl RobotControlInstance;
    return &RobotControlInstance;
}
#endif // ROBOTCONTROL_H
