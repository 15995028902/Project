#include "globalvars.h"

int gl_nDeviceType = 4; /* USBCAN-II */
int gl_nDeviceInd = 0; //设备索引号，比如当只有一个设备时，索引号为0，有两个时可以为0或1
int gl_nReserved =0;
int gl_nCANInd[2]={0,1};//第几路CAN。即对应卡的CAN通道号， CAN0为0， CAN1为1。

VCI_CAN_OBJ   gl_TransmitData_USBCAN;
VCI_CAN_OBJ   gl_ReceiveData_USBCAN;

DrvInfo *gl_drvInfo[6];//根据1个CAN节点所控制的驱动器的数量确定数组大小，该示例定义为6个电机驱动器

motorDataCollection gl_motordata;

ArmWorkCommand gl_ArmCommand_6Dofs;

DesiredValuesStruct gl_ArmDesiredAngles_6Dofs;//关节空间期望电机角度




