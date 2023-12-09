#include "mythread.h"

MyThread::MyThread()
{

}

void MyThread::StartMotion()
{
    //初始化ZLG-USBCAN卡
    Open_USBCANdevice();
    Init_USBCANdevice(0);
    Start_USBCANdevice(0);
    Init_USBCANdevice(1);
    Start_USBCANdevice(1);

    //定义定时器，控制电机运动
    QTimer  *timer1=new QTimer(this); //创建定时器 timer1
    connect(timer1,SIGNAL(timeout()),this,SLOT(timer1update()));
    timer1->start(10);//10ms的定时器，给电机发送控制位置指令

    //定义定时器，读取电机驱动器发来的状态信息
    QTimer  *timer2=new QTimer(this); //创建定时器 timer1
    connect(timer2,SIGNAL(timeout()),this,SLOT(timer2update()));
    timer2->start(1);//1ms的定时器
}


void MyThread::StopMotion()
{
    timer1->stop();
    delete timer1;
    timer2->stop();
    delete timer2;

    for(int i=1;i<=6; i++)    DrvBrake(0, i);//刹车
    msleep(10);//等待10ms，方便电机驱动器执行急停指令
    WaitForStop();//等待电机停止运动
    for(int i=1;i<=6; i++)//断使能
        DrvEnable(0, i);//第一个参数：1表示驱动器使能，0表示断使能；第二个参数：所要设置的电机驱动器的ID，1-6

    Close_USBCANdevice();//关闭ZLG-USBCAN卡
}


bool MyThread::WaitForStop()
{
    long len;
    int count=0;

wait:
    len = Receive_USBCANdevice(0);//读CAN0口，机械臂电机驱动器与CAN0口通信
    if(len!= 0xFFFFFFFF && len>0) // 判断是否接收到CAN信息.处理接收到的电机驱动器信息
        ResolveMotorData(0);//解析CAN0接收的机械臂1驱动器数据

    if( gl_drvInfo[0]->spd==0.0 && gl_drvInfo[1]->spd==0.0 && gl_drvInfo[2]->spd==0.0 && gl_drvInfo[3]->spd==0.0 && gl_drvInfo[4]->spd==0.0 && gl_drvInfo[5]->spd==0.0)
        return true;
    msleep(1);
    count++;
    if(count ==1000)  return false;//如果1s时间都没有实现急停，返回 false
    goto wait;
}


void MyThread::timer1update()
{
    static int i=0;
    static QTime lastTime_temp;
    if( i==0){
        lastTime_temp = QTime::currentTime();//用于第一次发送指令时的耗时计算
        i=1;
    }

    float tempData[6];

    for(int i=0;i<=5;i++)//给6个关节发送位置运动指令，电机默认工作在位置模式
    {
        if(gl_ArmDesiredAngles_6Dofs.Driv[i].size()>0)
        {
            DrvClrFlt(i+1);//清报警
            DrvEnable(1, i+1);//电机使能
            DrvMotion(gl_ArmDesiredAngles_6Dofs.Driv[i].at(0), i+1);//发送运动指令
            tempData[i] = gl_ArmDesiredAngles_6Dofs.Driv[i].at(0);
            gl_ArmDesiredAngles_6Dofs.Driv[i].erase(gl_ArmDesiredAngles_6Dofs.Driv[i].begin());
        }
        else return;//如果指令都已经发完了，就返回
    }

    //发送采集的电机驱动器信息给SaveThread，供其写入excel文件
    emit SensorsSignal(0, lastTime_temp.msecsTo(QTime::currentTime()),tempData[0], tempData[1],tempData[2],tempData[3],tempData[4],tempData[5] ); // 发送给SaveDataThread线程

    lastTime_temp = QTime::currentTime();//用于发送指令的耗时计算
}


void MyThread::timer2update()
{
    long len;
    static int i=0;
    uint client_id;//CAN节点接收到的电机驱动器信息的ID号

    if( i==0){
        m_lastTime = QTime::currentTime();//用于第一次采集驱动器信息时的耗时计算
        i=1;
    }

    len = Receive_USBCANdevice(0);//读CAN0口，机械臂电机驱动器与CAN0口通信
    if(len!= 0xFFFFFFFF && len>0) // 判断是否接收到CAN信息.处理接收到的电机驱动器信息
    {

        if ( ResolveMotorData(0) ==1 ) return;//解析CAN0接收的机械臂1驱动器数据，如果接收到的是伺服设置信息，不是位移/速度/电流等状态信息，就返回
        client_id=gl_ReceiveData_USBCAN.ID&0x0F -1;//提取驱动器的ID号并减1，该参数用于设置数组的序号

        m_lastTime.msecsTo(QTime::currentTime());
        //发送采集的电机驱动器信息给SaveThread，供其写入excel文件
        emit SensorsSignal(gl_ReceiveData_USBCAN.ID&0x0F, m_lastTime.msecsTo(QTime::currentTime()),gl_drvInfo[client_id]->pos, gl_drvInfo[client_id]->spd,gl_drvInfo[client_id]->ia,0,0,0 ); // 发送给SaveDataThread线程
    }

    m_lastTime = QTime::currentTime();//用于采集信号的耗时计算
}




#if 0
/**
 * @brief TRemoteNode::CalculateDesiredPosition
 * 计算电缸机械臂的各关节电机期望角度
 */
void MyThread::CalculateDesiredPosition()
{
    QByteArray BufferArray;
   // double theta_output[7];
    double paramater_output[6];
    Eigen::Matrix4d T06;

    if( gl_ArmCommand_6Dofs.x.size()==0 )  return;//如果没有接收到指令
    paramater_output[0] = ui->
    paramater_output[0] = gl_ArmCommand_6Dofs.x.at(0);
    paramater_output[1] = gl_ArmCommand_6Dofs.y.at(0);
    paramater_output[2] = gl_ArmCommand_6Dofs.z.at(0);
    paramater_output[3] = gl_ArmCommand_6Dofs.rx.at(0);
    paramater_output[4] = gl_ArmCommand_6Dofs.ry.at(0);
    paramater_output[5] = gl_ArmCommand_6Dofs.rz.at(0);

    printf("Endposture_1 is %f\n", paramater_output[0]);
    printf("Endposture_2 is %f\n", paramater_output[1]);
    printf("Endposture_3 is %f\n", paramater_output[2]);
    printf("Endposture_4 is %f\n", paramater_output[3]);
    printf("Endposture_5 is %f\n", paramater_output[4]);
    printf("Endposture_6 is %f\n", paramater_output[5]);

    if( gl_ArmCommand_6Dofs.x.size()>=2) gl_ArmCommand_6Dofs.x.erase(gl_ArmCommand_6Dofs.x.begin());
    if( gl_ArmCommand_6Dofs.y.size()>=2) gl_ArmCommand_6Dofs.y.erase(gl_ArmCommand_6Dofs.y.begin());
    if( gl_ArmCommand_6Dofs.z.size()>=2) gl_ArmCommand_6Dofs.z.erase(gl_ArmCommand_6Dofs.z.begin());
    if( gl_ArmCommand_6Dofs.rx.size()>=2) gl_ArmCommand_6Dofs.rx.erase(gl_ArmCommand_6Dofs.rx.begin());
    if( gl_ArmCommand_6Dofs.ry.size()>=2) gl_ArmCommand_6Dofs.ry.erase(gl_ArmCommand_6Dofs.ry.begin());
    if( gl_ArmCommand_6Dofs.rz.size()>=2) gl_ArmCommand_6Dofs.rz.erase(gl_ArmCommand_6Dofs.rz.begin());
//  if( gl_ArmCommand_6Dofs.Stop.size()>=2) gl_ArmCommand_6Dofs.Stop.erase(gl_FirstArmCommand_6Dofs.Stop.begin());

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




