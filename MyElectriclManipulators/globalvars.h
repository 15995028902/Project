#ifndef GLOBALVARS_H
#define GLOBALVARS_H

#include <QVector>
#include  "ControlCAN.h"

extern int gl_nDeviceType; /* USBCAN-II */
extern int gl_nDeviceInd;   //设备索引号，比如当只有一个设备时，索引号为0，有两个时依次为0和1
extern int gl_nReserved;
extern int gl_nCANInd[2];//第几路CAN。即对应卡的CAN通道号， CAN0为0， CAN1为1。

extern VCI_CAN_OBJ   gl_ReceiveData_USBCAN;//接收数据    VCI_CAN_OBJ 结构体是 CAN 帧结构体，即 1 个结构体表示一个帧的数据结构。
extern VCI_CAN_OBJ   gl_TransmitData_USBCAN;//发送数据


/*****电缸臂各个关节代码*****/
const int BaseJoint =1;//底座
const int BigArmJoint =2;//大臂
const int SecondArmJoint = 3;//二臂
const int RockerJoint = 4;//摇杆
const int SideJoint = 5;//末端侧摆
const int RotateJoint = 6;//末端回转


//定义结构体，保存电机及驱动器的相关状态信息
typedef struct
{
    float pos;//电机角度或电动缸位移
    float spd;//电机转速，单位r/min
    float ia;//电机电流
    float bv;//电机母线电压
    unsigned int enable;//电机驱动器使能状态
    unsigned int digit_in;//电机驱动器开关量输入状态
    unsigned int digit_out;//电机驱动器开关量输出状态
    unsigned int wrn;//电机驱动器报警信号
    unsigned int mode;//电机驱动器工作模式
    unsigned int src;//电机驱动器的指令源
    unsigned int id;//电机驱动器ID
}DrvInfo;

//定义联合体，该联合体用以CAN总线处理浮点型数据，根据IEEE标准，一个浮点型数据用4个字节进行表示
typedef union
{
    float float_data;
    unsigned char byte_data[4];
}float_byte;

typedef struct
{
    std::vector<float> m_time;
    std::vector<float> m_voltage;
    std::vector<float> m_current;
    std::vector<float> m_position;
    std::vector<float> m_speed;
}motorDataCollection;

//6Dofs液压臂的工作指令，末端期望位移，需要逆运动学解算为关节空间指令。从操作台发送至电缸臂
typedef struct
{
    std::vector<float> x; //1个double是8bytes
    std::vector<float> y;
    std::vector<float> z;
    std::vector<float> rx;
    std::vector<float> ry;
    std::vector<float> rz;

}ArmWorkCommand;

extern motorDataCollection gl_motordata;

extern ArmWorkCommand gl_ArmCommand_6Dofs;

extern DrvInfo *gl_drvInfo[6]; //根据1个CAN节点所控制的驱动器的数量确定数组大小，该示例定义为6个电机驱动器

struct DesiredValuesStruct{

    std::vector<float>  Driv[6];//电机1-6的角度
    std::vector<float> linearmotor;//自动换装电机的控制指令
    std::vector<float> toolCommand;//末端属具的控制指令
};
extern DesiredValuesStruct gl_ArmDesiredAngles_6Dofs;//关节空间期望电机角度

#endif // GLOBALVARS_H

