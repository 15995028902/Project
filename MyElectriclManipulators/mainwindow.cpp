#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_Enable_clicked()
{

}


void MainWindow::on_enable_3_clicked()
{

}

/**
 * @brief MainWindow::on_pushButton_clicked
 * 单关节运动测试
 */
void MainWindow::on_pushButton_clicked()
{
    if( ui->pushButton->text()=="开始运动" )
    {
        //初始化ZLG-USBCAN卡
        Open_USBCANdevice();
        Init_USBCANdevice(0);
        Start_USBCANdevice(0);
        Init_USBCANdevice(1);
        Start_USBCANdevice(1);

        DrvClrFlt(ui->comboBox->currentIndex()+1);//清报警
        DrvEnable(1, ui->comboBox->currentIndex()+1);//电机使能
        DrvMotion(ui->doubleSpinBox->value(), ui->comboBox->currentIndex()+1);//发送运动指令

        ui->pushButton->setText("停止运动");
    }

    else if( ui->pushButton->text()=="停止运动" )
    {
        DrvBrake(0, ui->comboBox->currentIndex()+1);//刹车
        DrvEnable(0, ui->comboBox->currentIndex()+1);//第一个参数：1表示驱动器使能，0表示断使能；第二个参数：所要设置的电机驱动器的ID，1-6

        Close_USBCANdevice();//关闭ZLG-USBCAN卡
        ui->pushButton->setText("开始运动");
    }
}

/**
 * @brief MainWindow::on_pushButton_2_clicked
 * 读取机械臂当前状态信息
 */
void MainWindow::on_pushButton_2_clicked()
{
    //初始化ZLG-USBCAN卡
    Open_USBCANdevice();
    Init_USBCANdevice(0);
    Start_USBCANdevice(0);
    Init_USBCANdevice(1);
    Start_USBCANdevice(1);
    bool i1=false, i2=false, i3=false,i4=false,i5=false,i6=false;

    int count=0;

    do{
        int len = Receive_USBCANdevice(0);//读CAN0口，机械臂电机驱动器与CAN0口通信
        if(len!= 0xFFFFFFFF && len>0) // 判断是否接收到CAN信息.处理接收到的电机驱动器信息
        {
            if ( ResolveMotorData(0) ==1 ) continue;//解析CAN0接收的机械臂1驱动器数据，如果接收到的是伺服设置信息，不是位移/速度/电流等状态信息，跳过本次循环，进行下次循环
            int client_id=gl_ReceiveData_USBCAN.ID&0x0F ;//提取驱动器的ID号

            switch (client_id) {
            case 1:
                i1=true;
                ui->doubleSpinBox_2->setValue(gl_drvInfo[client_id-1]->pos);
                break;
            case 2:
                i2=true;
                ui->doubleSpinBox_3->setValue(gl_drvInfo[client_id-1]->pos);
                break;
            case 3:
                i2=true;
                ui->doubleSpinBox_4->setValue(gl_drvInfo[client_id-1]->pos);
                break;
            case 4:
                i2=true;
                ui->doubleSpinBox_5->setValue(gl_drvInfo[client_id-1]->pos);
                break;
            case 5:
                i2=true;
                ui->doubleSpinBox_6->setValue(gl_drvInfo[client_id-1]->pos);
                break;
            case 6:
                i2=true;
                ui->doubleSpinBox_7->setValue(gl_drvInfo[client_id-1]->pos);
                break;
            }
        }

        count++;
        if(count == 200)     {
            QMessageBox::warning(this, tr("警告对话框"), tr("读取机械臂信息错误"), QMessageBox::Yes,QMessageBox::No);
            return;
        }

    }while(! (i1 && i2 && i3 && i4 && i5 &&i6));

    //计算机械臂当前末端位姿
    Electrickinematics endPosture_temp;
    Eigen::Matrix4d T06_output;
    double theta_input[6];
    theta_input[0]=ui->doubleSpinBox_2->value();
    theta_input[1]=ui->doubleSpinBox_3->value();
    theta_input[2]=ui->doubleSpinBox_4->value();
    theta_input[3]=ui->doubleSpinBox_5->value();
    theta_input[4]=ui->doubleSpinBox_6->value();
    theta_input[5]=ui->doubleSpinBox_7->value();
    endPosture_temp.kinematics(theta_input, &T06_output);//正运动学，计算末端位姿矩阵
    //将末端位姿矩阵转化为 X Y Z RX RY RZ
    double  X,Y,Z,RX,RY,RZ;
    //补充：将T06_output转化为末端的 X Y Z RX RY RZ

    ui->doubleSpinBox_8->setValue(0);
    ui->doubleSpinBox_9->setValue(0);
    ui->doubleSpinBox_10->setValue(0);
    ui->doubleSpinBox_11->setValue(0);
    ui->doubleSpinBox_12->setValue(0);
    ui->doubleSpinBox_13->setValue(0);

    Close_USBCANdevice();//关闭ZLG-USBCAN卡
}

/**
 * @brief MainWindow::on_pushButton_4_clicked
 * 机械臂末端转化为关节量
 */
void MainWindow::on_pushButton_4_clicked()
{
    Electrickinematics endPosture_temp;
    Eigen::Matrix4d T06_output;
    double XPos_input = ui->doubleSpinBox_14->value();
    double YPos_input = ui->doubleSpinBox_15->value();
    double ZPos_input = ui->doubleSpinBox_16->value();
    double XRotate_input = ui->doubleSpinBox_16->value();
    double YRotate_input = ui->doubleSpinBox_17->value();
    double ZRotate_input = ui->doubleSpinBox_18->value();

    //根据末端X Y Z RX RY RZ 计算末端位姿矩阵
    endPosture_temp.kinematics2(&XPos_input, &YPos_input, &ZPos_input, &XRotate_input, &YRotate_input, &ZRotate_input, &T06_output);

    Eigen::Matrix4d T06_input;
    T06_input=T06_output;
    double theta_output[6];
    endPosture_temp.inverse_kinematics(&T06_input,theta_output);//转化为关节角

    ui->doubleSpinBox_20->setValue(theta_output[0]);
    ui->doubleSpinBox_21->setValue(theta_output[1]);
    ui->doubleSpinBox_22->setValue(theta_output[2]);
    ui->doubleSpinBox_23->setValue(theta_output[3]);
    ui->doubleSpinBox_24->setValue(theta_output[4]);
    ui->doubleSpinBox_25->setValue(theta_output[5]);

}

/**
 * @brief MainWindow::on_pushButton_3_clicked
 * 开始所有关节联动
 */
void MainWindow::on_pushButton_3_clicked()
{

    {/***轨迹规划---将期望位移与当前位移之间的距离，划分为 10000个 线段****/
        on_pushButton_2_clicked();//读取机械臂当前位姿信息
        on_pushButton_4_clicked();//将目标位姿转化为期望关节角度

        for (int i=0;i<=5;i++) {
            gl_ArmDesiredAngles_6Dofs.Driv[i].clear();
        }
        int segments=10000;
        float BaseSampled =  ui->doubleSpinBox_8->value();
        float BaseDesired = ui->doubleSpinBox_20->value();
        float increament= (BaseDesired -  BaseSampled )/segments;
        for (int i=1; i<=segments; i++) {

            gl_ArmDesiredAngles_6Dofs.Driv[0].push_back(BaseSampled+increament);
        }
        float BigArmSampled =  ui->doubleSpinBox_9->value();
        float BigArmDesired = ui->doubleSpinBox_21->value();
        increament= (BigArmDesired -  BigArmSampled )/segments;
        for (int i=1; i<=segments; i++) {
            gl_ArmDesiredAngles_6Dofs.Driv[1].push_back(BigArmSampled+increament);
        }

        float SecondArmSampled =  ui->doubleSpinBox_10->value();
        float SecondArmDesired = ui->doubleSpinBox_22->value();
        increament= (SecondArmDesired -  SecondArmSampled )/segments;
        for (int i=1; i<=segments; i++) {
            gl_ArmDesiredAngles_6Dofs.Driv[2].push_back(SecondArmSampled+increament);
        }

        float RockerSampled =  ui->doubleSpinBox_11->value();
        float RockerDesired = ui->doubleSpinBox_23->value();
        increament= (RockerDesired -  RockerSampled )/segments;
        for (int i=1; i<=segments; i++) {
            gl_ArmDesiredAngles_6Dofs.Driv[3].push_back(RockerSampled+increament);
        }

        float SideSampled =  ui->doubleSpinBox_12->value();
        float SideDesired = ui->doubleSpinBox_24->value();
        increament= (SideDesired -  SideSampled )/segments;
        for (int i=1; i<=segments; i++) {
            gl_ArmDesiredAngles_6Dofs.Driv[4].push_back(SideSampled+increament);
        }

        float RotateSampled =  ui->doubleSpinBox_13->value();
        float RotateDesired = ui->doubleSpinBox_25->value();
        increament= (RotateDesired -  RotateSampled )/segments;
        for (int i=1; i<=segments; i++) {
            gl_ArmDesiredAngles_6Dofs.Driv[5].push_back(RotateSampled+increament);
        }
    }

    if( ui->pushButton_3->text()=="开始联动" )
    {
        QObject::connect(&m_myThread, SIGNAL( SensorsSignal(int, int,float, float,float,float,float,float ) ), &m_SaveDataThread, SLOT( WriteXlsxSlot(int, int,float, float,float,float,float,float) )); // 注意double后面都没有形参
        m_myThread.start();
        m_myThread.StartMotion();
        m_SaveDataThread.start();

        ui->pushButton_3->setText("停止运动");
    }

    else if( ui->pushButton_3->text()=="停止运动" )
    {
        m_myThread.StopMotion();
        m_myThread.terminate();
        m_myThread.wait();
        m_SaveDataThread.terminate();
        m_SaveDataThread.wait();
        QObject::disconnect(&m_myThread, SIGNAL( SensorsSignal(int, int,float, float,float,float,float,float ) ), &m_SaveDataThread, SLOT( WriteXlsxSlot(int, int,float, float,float,float,float,float) )); // 注意double后面都没有形参

        ui->pushButton_3->setText("开始联动");
    }
}
