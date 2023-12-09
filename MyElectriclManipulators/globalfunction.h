#ifndef GLOBALFUNCTION_H
#define GLOBALFUNCTION_H

#include "ControlCAN.h"
#include "globalvars.h"

#include <Eigen/Eigen>
using namespace Eigen;


//ZLGCan 函数块
bool Open_USBCANdevice();
bool Close_USBCANdevice();
bool Init_USBCANdevice(int i);
bool Start_USBCANdevice(int i);

long Receive_USBCANdevice(int i);
long Transmit_USBCANdevice(int i);


//电缸控制函数块
void DrvEnable(unsigned int v,unsigned int drv_id); //驱动器使能函数，参数v=1表示驱动器使能，v=0表示断使能；参数drv_id为所要设置的电机驱动器的ID
void DrvMonitor(unsigned int v,unsigned int drv_id);//驱动器状态监控函数，参数v=1表示使能驱动器状态监控，v=0表示禁止驱动器上传状态信息；参数drv_id为所要设置的电机驱动器的ID
void DrvRunMode(unsigned int run_mode,unsigned int drv_id);//驱动器工作模式设置函数，参数run_mode=3表示位置模式，=2表示速度模式，=0表示电流模式；参数drv_id为所要设置的电机驱动器的ID
void DrvBrake(unsigned int v,unsigned int drv_id);//电机制动器控制函数，参数v=1表示打开制动器，电机轴可以转动，v=0表示关闭制动器，电机轴刹车；参数drv_id为所要设置的电机驱动器的ID
void DrvMotion(float val,unsigned int drv_id);//电机或电动缸运动控制函数，参数val表示运动的数值，该数值的具体含义由电机驱动器的工作模式决定，分别可表示为角度(°)或位移（mm）、速度（r/min），电流（A）；参数drv_id为所要设置的电机驱动器的ID
void DrvClrFlt(unsigned int drv_id);//故障清除函数，参数v=1表示清除电机驱动器故障；参数drv_id为所要设置的电机驱动器的ID


int ResolveMotorData( int chanel);

#endif // GLOBALFUNCTION_H
