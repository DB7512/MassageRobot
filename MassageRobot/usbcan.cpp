#include "usbcan.h"
#include <QDebug>
#include <unistd.h>
#include <QTime>
#include <QCoreApplication>
#include <QVariant>
#include "robotcontrol.h"
USBCAN::USBCAN()
{
    stopped = false;
}

USBCAN::~USBCAN()
{
    closeDevice();
}

bool USBCAN::openDevice(unsigned int deviceType, unsigned int debicIndex, unsigned int baundRate)
{
    stopped = false;
    m_deviceType = deviceType;/* USBCAN-2A或USBCAN-2C或CANalyst-II */
    m_debicIndex = debicIndex;/* 第1个设备 */
    m_baundRate = baundRate;
    unsigned int dwRel;
    dwRel = VCI_OpenDevice(m_deviceType, m_debicIndex, 0);
    if(dwRel != 1)
        return false;
    else
        qDebug()<<"open success";
    return true;
}

bool USBCAN::initCAN()
{
    unsigned int dwRel = VCI_ClearBuffer(m_deviceType, m_debicIndex, 0);
    dwRel = VCI_ClearBuffer(m_deviceType, m_debicIndex, 1);
    VCI_INIT_CONFIG vic;
    vic.AccCode=0x80000008;
    vic.AccMask=0xFFFFFFFF;
    vic.Filter=1;
    vic.Mode=0;
    switch (m_baundRate) {
    case 10:
        vic.Timing0=0x31;
        vic.Timing1=0x1c;
        break;
    case 20:
        vic.Timing0=0x18;
        vic.Timing1=0x1c;
        break;
    case 40:
        vic.Timing0=0x87;
        vic.Timing1=0xff;
        break;
    case 50:
        vic.Timing0=0x09;
        vic.Timing1=0x1c;
        break;
    case 80:
        vic.Timing0=0x83;
        vic.Timing1=0xff;
        break;
    case 100:
        vic.Timing0=0x04;
        vic.Timing1=0x1c;
        break;
    case 125:
        vic.Timing0=0x03;
        vic.Timing1=0x1c;
        break;
    case 200:
        vic.Timing0=0x81;
        vic.Timing1=0xfa;
        break;
    case 250:
        vic.Timing0=0x01;
        vic.Timing1=0x1c;
        break;
    case 400:
        vic.Timing0=0x80;
        vic.Timing1=0xfa;
        break;
    case 500:
        vic.Timing0=0x00;
        vic.Timing1=0x1c;
        break;
    case 666:
        vic.Timing0=0x80;
        vic.Timing1=0xb6;
        break;
    case 800:
        vic.Timing0=0x00;
        vic.Timing1=0x16;
        break;
    case 1000:
        vic.Timing0=0x00;
        vic.Timing1=0x14;
        break;
    case 33:
        vic.Timing0=0x09;
        vic.Timing1=0x6f;
        break;
    case 66:
        vic.Timing0=0x04;
        vic.Timing1=0x6f;
        break;
    case 83:
        vic.Timing0=0x03;
        vic.Timing1=0x6f;
        break;
    default:
        break;
    }
    dwRel = VCI_InitCAN(m_deviceType, m_debicIndex, 0, &vic);
    dwRel = VCI_InitCAN(m_deviceType, m_debicIndex, 1, &vic);
    if(dwRel !=1)
        return false;
    else
        qDebug()<<"init success";

    VCI_BOARD_INFO vbi;
    //获取设备信息
    dwRel = VCI_ReadBoardInfo(m_deviceType, m_debicIndex, &vbi);
    if(dwRel != 1)
        return false;
    else
        return true;
}

bool USBCAN::startCAN()
{
    if(VCI_StartCAN(m_deviceType, m_debicIndex, 0) !=1)
    {
        qDebug()<<"start 0 fail.";
        return false;
    } else {
        qDebug()<<"start 0 success.";
    }
    if(VCI_StartCAN(m_deviceType, m_debicIndex, 1) !=1)
    {
        qDebug()<<"start 1 fail.";
        return false;
    } else {
        qDebug()<<"start 1 success.";
    }
    return true;
}

void USBCAN::closeDevice()
{
    stopped = true;
    VCI_CloseDevice(m_deviceType, m_debicIndex);
    qDebug("CAN已关闭。");
}

bool USBCAN::initSensor()
{
    if(stopSensor()) {
        if(clearSensor()) {
            if(autoSensor()) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    } else {
        return false;
    }
}

bool USBCAN::stopSensor()
{
    unsigned int dwRel;
    unsigned char data[7] = {0x00, 0xab, 0x06, 0x9C, 0x47, 0x00, 0x00};
    VCI_CAN_OBJ vco;
    vco.ID = 1 ;
    vco.RemoteFlag = 0;//数据帧
    vco.ExternFlag = 0;//标准帧
    vco.DataLen = 7;
    for(UINT j = 0; j < 7;j++)
        vco.Data[j] = data[j];
    dwRel = VCI_Transmit(m_deviceType, m_debicIndex, 0, &vco, 1);
    if(dwRel>0)
        return true;
    else
        return false;
}

bool USBCAN::clearSensor()
{
    unsigned int dwRel;
    unsigned char data[7] = {0x00, 0xab, 0x06, 0x9C, 0x48, 0x00, 0xaa};
    VCI_CAN_OBJ vco;
    vco.ID = 1 ;
    vco.RemoteFlag = 0;//数据帧
    vco.ExternFlag = 0;//标准帧
    vco.DataLen = 7;
    for(UINT j = 0; j < 7;j++)
        vco.Data[j] = data[j];
    dwRel = VCI_Transmit(m_deviceType, m_debicIndex, 0, &vco, 1);
    if(dwRel>0)
        return true;
    else
        return false;
}

bool USBCAN::autoSensor()
{
    unsigned int dwRel;
    unsigned char data[7] = {0x00, 0xab, 0x06, 0x9C, 0x47, 0x00, 0x01};
    VCI_CAN_OBJ vco;
    vco.ID = 1 ;
    vco.RemoteFlag = 0;//数据帧
    vco.ExternFlag = 0;//标准帧
    vco.DataLen = 7;
    for(UINT j = 0; j < 7;j++)
        vco.Data[j] = data[j];
    dwRel = VCI_Transmit(m_deviceType, m_debicIndex, 0, &vco, 1);
    if(dwRel>0)
        return true;
    else
        return false;
}

void USBCAN::run()
{
    float avg = 0.0;
    float lastavg = 0.0;
    int lastmaxforce = 0.0;
    int maxforce = 0;
    int minforce = 0;
    while(!stopped)
    {
        DWORD dwRel;
        VCI_CAN_OBJ vco[2500];
        QString str = "";
        int dechigh = 0, declow = 0;
        int threshold = 56797;//0xDDDD
        int force = 0;
        int sum = 0;
        dwRel = VCI_Receive(m_deviceType, m_debicIndex, 0, vco,2500,0);
        //qDebug()<<"derel"<<dwRel;
        if(dwRel > 0 && dwRel < 2501) {
            lastavg = avg;
            int num = dwRel;
            minforce = threshold;
            for(unsigned int i = 0; i < dwRel; i++) {
                str = vco[i].Data[6];
                dechigh = QVariant(vco[i].Data[5]).toInt();
                declow = QVariant(vco[i].Data[6]).toInt();
                force = dechigh*255 + declow;
                //qDebug()<<"force"<<force;
                if(force > maxforce && force < threshold) {
                    maxforce = force;
                }
                if(force < minforce) {
                    minforce = force;
                }
                if(force > threshold) {
                    num --;
                } else {
                    sum += force;
                }
            }
            if(num > 2) {
                avg = (sum - minforce - maxforce) / (num - 2);
            } else {
                avg = lastavg;
                maxforce = lastmaxforce;
            }
            lastmaxforce = maxforce;
        }else if(dwRel > 2500) {
            qDebug("CAN RECEIVE ERROR！\n");
        }
        m_sensorMessage = avg/10.0;
        emit getSensorData(m_sensorMessage);
        maxforce = 0;
        QThread::msleep(5);
    }
}

void USBCAN::sleep(int msec)
{
    QTime dieTime = QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < dieTime )
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}
