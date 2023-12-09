#include "globalfunction.h"
#include <QMessageBox>


/**********************************************************
 *
 * 这部分为USNB-CAN设备的操作函数
 *
 * ********************************************************/
bool Open_USBCANdevice()
{
    DWORD dwRel;
    dwRel = VCI_OpenDevice(gl_nDeviceType, gl_nDeviceInd, gl_nReserved);
    //qDebug()<<dwRel<<gl_nDeviceType<<gl_nDeviceInd<<gl_nReserved;
    if (dwRel != STATUS_OK)
    {
        QMessageBox::critical(0, QObject::tr("打开设备失败!"),
                              "打开设备失败!！ ", QMessageBox::Ok);
        return false;
    }
    return true;
}

bool Close_USBCANdevice()
{
    DWORD dwRel;
    dwRel = VCI_CloseDevice(gl_nDeviceType, gl_nDeviceInd);
    //qDebug()<<dwRel;
    if (dwRel != STATUS_OK)
    {
        QMessageBox::critical(0, QObject::tr("关闭设备失败!"),
                              "关闭设备失败!！ ", QMessageBox::Ok);
        return false;
    }
    return true;
}

bool Init_USBCANdevice( int i )
{
    VCI_INIT_CONFIG init_config;
    init_config.AccCode = 0;//验收码
    init_config.AccMask = 0xffffffff;//屏蔽码推荐设置为 0xFFFFFFFF，即全部接收
    init_config.Filter = 0; //滤波模式: 0/1:接收所有类型;2:只接收标准帧;3:只接收
    init_config.Timing0 = 0x00;
    init_config.Timing1 = 0x1c;//Timing0 和 Timing1设置波特率,500Kbps
    init_config.Mode = 0;// 0-正常模式，1-只听模式，2-自发自收模式

    DWORD dwRel;
    dwRel = VCI_InitCAN(gl_nDeviceType, gl_nDeviceInd, gl_nCANInd[i], &init_config);//只初始化通道0;即CAN1
    //qDebug()<<dwRel;
    //dwRel = VCI_InitCAN(gl_nDeviceType, gl_nDeviceInd, gl_nCANInd[1], &init_config);//初始化通道0和通道1;即CAN1和CAN2
    if (dwRel == STATUS_ERR)
    {
            QMessageBox::critical(0, QObject::tr("初始化设备失败!"),
            "初始化设备失败!！ ", QMessageBox::Cancel);
           return false;
    }
    return true;
}

bool Start_USBCANdevice( int i )
{
    DWORD dwRel;
    dwRel = VCI_StartCAN(gl_nDeviceType, gl_nDeviceInd, gl_nCANInd[i]);

    if (dwRel == STATUS_ERR)
    {
        QMessageBox::critical(0, QObject::tr("启动设备失败!"),
        "启动设备失败!！ ", QMessageBox::Cancel);
       return false;
    }
    return true;
}

long Receive_USBCANdevice(int i )
{
    DWORD dwRel;
    //VCI_CAN_OBJ vco[100];
    //Q_UNUSED(vco);
    dwRel =VCI_Receive(gl_nDeviceType, gl_nDeviceInd, gl_nCANInd[i], &gl_ReceiveData_USBCAN, 1, 0);//接收1帧
    return dwRel;
}

long  Transmit_USBCANdevice( int i )
{
    DWORD dwRel;
    dwRel = VCI_Transmit(gl_nDeviceType, gl_nDeviceInd, gl_nCANInd[i], &gl_TransmitData_USBCAN, 1);//发送1帧
    return dwRel;
}


/**********************************************************
 *
 * 这部分为电机驱动器设备的操作函数
 *
 * ********************************************************/

/*电机驱动器使能函数*/
void DrvEnable(unsigned int v, unsigned int drv_id)
//参数v：1 表示驱动器使能，0 表示驱动器断使能；
//参数drv_id表示驱动器的ID号，范围：1...client_num；
{
    gl_TransmitData_USBCAN.ID = 0x400+drv_id;//报文ID
    gl_TransmitData_USBCAN.Data[0] = '3';
    gl_TransmitData_USBCAN.Data[1] = '2';
    gl_TransmitData_USBCAN.Data[4] = v;   
    Transmit_USBCANdevice( 0 );//CAN0发送
}

/*电机驱动器是否回传信息设置函数*/
void DrvMonitor(unsigned int v, unsigned int drv_id)
//参数v：1 表示上传驱动器的状态信息，0 表示禁止上传驱动器的状态信息，状态信息包括：驱动器开关量信息、报警信息、电机电流、转速、位置
//参数drv_id表示驱动器的ID号，范围：1...client_num；
{
    gl_TransmitData_USBCAN.ID=0x400 + drv_id; //报文ID
    gl_TransmitData_USBCAN.Data[0]='8';
    gl_TransmitData_USBCAN.Data[1]='0';
    gl_TransmitData_USBCAN.Data[4]=v;
    Transmit_USBCANdevice( 0 );//CAN0发送
}


/*电机驱动器工作模式设置函数*/
void DrvRunMode(unsigned int run_mode, unsigned int drv_id)
//参数run_mode：0 表示驱动器工作在电流环，2 表示驱动器工作在速度环，3 表示驱动器工作在位置环
//参数drv_id表示驱动器的ID号，范围：1...client_num；
{
  //  if (gl_drvInfo[drv_id-1]->enable) return;
    if (run_mode<4)
    {
        gl_TransmitData_USBCAN.ID=0x400 + drv_id; //报文ID
        gl_TransmitData_USBCAN.Data[0]='3';
        gl_TransmitData_USBCAN.Data[1]='0';
        gl_TransmitData_USBCAN.Data[4]=run_mode;
        Transmit_USBCANdevice( 0 );//CAN0发送
    }
}

/*电机抱闸设置函数*/
//参数v：1 表示电机抱闸打开(不抱闸)，0 表示电机抱闸关闭(抱闸)
//参数drv_id表示驱动器的ID号，范围：1...client_num；
void DrvBrake(unsigned int v, unsigned int drv_id)
{
    gl_TransmitData_USBCAN.ID=0x400 + drv_id; //报文ID
    gl_TransmitData_USBCAN.Data[0]='7';
    gl_TransmitData_USBCAN.Data[1]='0';
    gl_TransmitData_USBCAN.Data[4]=v;
    Transmit_USBCANdevice( 0 );//CAN0发送
}

/*电机运动指令发送函数*/
void DrvMotion(float val, uint drv_id)
//参数val表示电机的运动指令数据：
//如果电机工作在电流模式下，该数据则代表电机的驱动电流（单位：A）；
//如果电机工作在速度模式下，该数据则代表电机的转速（单位：r/min）；
//如果电机工作在位置模式下，该数据则代表电机的角度（单位：度）或者电动缸的位移（单位：mm）；
//参数drv_id表示驱动器的ID号，范围：1...client_num；
{
    float_byte a;
    a.float_data=val;
    gl_TransmitData_USBCAN.ID=0x400 + drv_id; //报文ID
    gl_TransmitData_USBCAN.Data[0]='5';
    gl_TransmitData_USBCAN.Data[1]='2';
    for (int i = 0; i < 4; ++i) gl_TransmitData_USBCAN.Data[7 - i] = a.byte_data[i];
     Transmit_USBCANdevice( 0 );//CAN0发送
}

/*电机驱动器故障清除设置函数*/
void DrvClrFlt(uint drv_id)
//参数drv_id表示驱动器的ID号，范围：1...client_num；
{
   // if (gl_drvInfo[drv_id-1]->enable) return;
    gl_TransmitData_USBCAN.ID=0x400 + drv_id; //报文ID
    gl_TransmitData_USBCAN.Data[0]='5';
    gl_TransmitData_USBCAN.Data[1]='4';
    Transmit_USBCANdevice( 0 );//CAN0发送
}


/**
 * @brief TRemoteNode::ResolveMotorData
 * 解析电机发来的状态数据，转化为实际的物理量
 * @param chanel CAN0 or CAN1
 */
int ResolveMotorData( int chanel)
{
    float_byte fltbt;

    uint frame_id; //CAN节点接收到的电机驱动器信息的帧ID
    uint client_id;//CAN节点接收到的电机驱动器信息的ID号

    client_id=gl_ReceiveData_USBCAN.ID&0x0F -1;//提取驱动器的ID号并减1，该参数用于设置数组的序号
    frame_id = client_id&0xFF0; //提取CAN信息帧ID的高8位

    if(frame_id==0x640) //数据包1，伺服设置信息
    {
        //CAN数据帧的字节0和1为驱动器的报警信息，wrn=0x1表示母线电压高报警，=0x2表示母线电压低报警，=0x4表示过电流报警，=0x8表示功放模块报警，=0x10表示超速报警
        gl_drvInfo[client_id]->wrn=gl_ReceiveData_USBCAN.Data[1]*256 + gl_ReceiveData_USBCAN.Data[0];

        //上述报警信息，除功放模块报警外，其余报警信息皆可以通过DrvClrFlt(uint drv_id)函数清除
        gl_drvInfo[client_id]->enable = gl_ReceiveData_USBCAN.Data[2];//CAN数据帧的字节2为驱动器是否使能信息， =1表示驱动器已经使能，=0表示断开使能

        //drv_info[client_id]->digit_in=can->rec_frame.data[3];//CAN数据帧的字节3为驱动器开关量输入的状态，本应用不使用
        //drv_info[client_id]->digit_out=can->rec_frame.data[4];//CAN数据帧的字节4为驱动器开关量输出的状态，本应用不使用
        //drv_info[client_id]->mode=can->rec_frame.data[5];//CAN数据帧的字节5为驱动器工作模式信息，rec_frame.data[5]=0表示电流模式，=2表示速度模式，=3表示位置模式
        gl_drvInfo[client_id]->mode = gl_ReceiveData_USBCAN.Data[5];//CAN数据帧的字节5为驱动器工作模式信息，rec_frame.data[5]=0表示电流模式，=2表示速度模式，=3表示位置模式

        //CAN数据帧的字节6保留
        gl_drvInfo[client_id]->bv = gl_ReceiveData_USBCAN.Data[7]*2;//CAN数据帧的字节7为驱动器母线电压数据

        return 1;
    }

    else if(frame_id==0x440) //数据包2，位移/速度/电流信息
    {
        for (int i = 0; i < 4; ++i)  fltbt.byte_data[i]=gl_ReceiveData_USBCAN.Data[3-i];//fltbt.byte_data[i]=can->send_frame.data[3 - i] ; //CAN数据帧的字节0-3为电机位置数据，采用4个字节的二进制数据表示，符合IEEE标准
        gl_drvInfo[client_id]->pos=fltbt.float_data;//将二进制数据转换为浮点数
        //drv_info[client_id]->ia=sqrt((can->rec_frame.data[4]*256+can->rec_frame.data[5])*0.01）);//CAN数据帧的字节4和5为驱动器A相电流数据, A
        gl_drvInfo[client_id]->ia=sqrt((gl_ReceiveData_USBCAN.Data[4]*256+gl_ReceiveData_USBCAN.Data[5])*0.01);//CAN数据帧的字节4和5为驱动器A相电流数据, A
        //drv_info[client_id]->spd=can->rec_frame.data[6]*256+can->rec_frame.data[7]-32000;//CAN数据帧的字节6和7为电机转速数据。rpm
        gl_drvInfo[client_id]->spd=gl_ReceiveData_USBCAN.Data[6]*256+gl_ReceiveData_USBCAN.Data[7]-32000;//CAN数据帧的字节6和7为电机转速数据。rpm
       return 2;
    }

}



#if 0
/**********************************************************
 *
 * 这部分为计算电缸机械臂的各关节电机期望角度
 *
 * ********************************************************/

void CalculateDesiredPosition_FirstRobot()
{
    QByteArray BufferArray;
   // double theta_output[7];
    double paramater_output[7];
    Eigen::Matrix4d T06;

    if( gl_FirstArmCommand_6Dofs.x.size()==0 )  return;//如果没有接收到指令

    paramater_output[1] = gl_FirstArmCommand_6Dofs.x.at(0);
    paramater_output[2] = gl_FirstArmCommand_6Dofs.y.at(0);
    paramater_output[3] = gl_FirstArmCommand_6Dofs.z.at(0);
    paramater_output[4] = gl_FirstArmCommand_6Dofs.rx.at(0);
    paramater_output[5] = gl_FirstArmCommand_6Dofs.ry.at(0);
    paramater_output[6] = gl_FirstArmCommand_6Dofs.rz.at(0);

    printf("Endposture_1 is %f\n", paramater_output[1]);
    printf("Endposture_2 is %f\n", paramater_output[2]);
    printf("Endposture_3 is %f\n", paramater_output[3]);
    printf("Endposture_4 is %f\n", paramater_output[4]);
    printf("Endposture_5 is %f\n", paramater_output[5]);
    printf("Endposture_6 is %f\n", paramater_output[6]);

    if( gl_FirstArmCommand_6Dofs.x.size()>=2) gl_FirstArmCommand_6Dofs.x.erase(gl_FirstArmCommand_6Dofs.x.begin());
    if( gl_FirstArmCommand_6Dofs.y.size()>=2) gl_FirstArmCommand_6Dofs.y.erase(gl_FirstArmCommand_6Dofs.y.begin());
    if( gl_FirstArmCommand_6Dofs.z.size()>=2) gl_FirstArmCommand_6Dofs.z.erase(gl_FirstArmCommand_6Dofs.z.begin());
    if( gl_FirstArmCommand_6Dofs.rx.size()>=2) gl_FirstArmCommand_6Dofs.rx.erase(gl_FirstArmCommand_6Dofs.rx.begin());
    if( gl_FirstArmCommand_6Dofs.ry.size()>=2) gl_FirstArmCommand_6Dofs.ry.erase(gl_FirstArmCommand_6Dofs.ry.begin());
    if( gl_FirstArmCommand_6Dofs.rz.size()>=2) gl_FirstArmCommand_6Dofs.rz.erase(gl_FirstArmCommand_6Dofs.rz.begin());
//   if( gl_FirstArmCommand_6Dofs.linearmotor.size()>=2) gl_FirstArmCommand_6Dofs.linearmotor.erase(gl_FirstArmCommand_6Dofs.linearmotor.begin());
//    if( gl_FirstArmCommand_6Dofs.toolCommand.size()>=2) gl_FirstArmCommand_6Dofs.toolCommand.erase(gl_FirstArmCommand_6Dofs.toolCommand.begin());
    if( gl_FirstArmCommand_6Dofs.Stop.size()>=2) gl_FirstArmCommand_6Dofs.Stop.erase(gl_FirstArmCommand_6Dofs.Stop.begin());

    //待补充。把计算的电机角度存到m_FirstArmDesired_6Dofs  m_SecondArmDesired_6Dofs变量中
    //将手柄发来的末端姿态转换为nx ny nz....ox oy oz
   // m_FirstRobot_6Dofs.kinematics2(&Endposture_X, &Endposture_Y, &Endposture_Z, &Endposture_RX, &Endposture_RY, &Endposture_RZ, &T06);

   // if( !m_FirstRobot_6Dofs.inverse_kinematics(&T06,theta_output) ) return;

   // m_FirstRobot_6Dofs.angleToDisplacement(theta_output, paramater_output);
    for(int i=0; i<=5; i++)
    {
        if(paramater_output[i+1]+initPOS1[i] >firstArm_maxValue[i] )
            m_ArmDesiredAngles_6Dofs.Driv[i].push_back(firstArm_maxValue[i]);
        else if(paramater_output[i+1]+initPOS1[i]  < firstArm_minValue[i])
            m_ArmDesiredAngles_6Dofs.Driv[i].push_back(firstArm_minValue[i]);
        else
           m_ArmDesiredAngles_6Dofs.Driv[i].push_back(paramater_output[i+1]+initPOS1[i]);
    }

}
#endif




