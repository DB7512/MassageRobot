#ifndef USBCAN_H
#define USBCAN_H

#include <QThread>
#include "controlcan.h"
#include <vector>
using namespace std;
class USBCAN : public QThread
{
    Q_OBJECT
public:

    USBCAN();
    ~USBCAN();
    //1.打开设备
    bool openDevice(UINT deviceType,UINT debicIndex,UINT baundRate);
    //2.初始化CAN
    bool initCAN();
    //3.启动CAN
    bool startCAN();
    //5.关闭设备
    void closeDevice();
    bool initSensor();
    bool stopSensor();
    bool clearSensor();
    bool autoSensor();

    UINT m_deviceType;
    UINT m_debicIndex;
    UINT m_baundRate;
    UINT m_debicCom;

    bool stopped;
    int count;
    float m_sensorMessage;
    std::vector<float> m_canMessage;

signals:
    void getSensorData(float Data);

private:
    void run();
    void sleep(int msec);

};

#endif // USBCAN_H
