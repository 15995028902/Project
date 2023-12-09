#include "electrickinematics.h"

const double d[6+1] = { 0.0,100.0, 0.0, 0.0, 0.0, 154.0, 150.0 };//两杆x轴之间的距离。为了和DH公式的下标对应，0不用，d[1]-d[6]可用
const double a[6] = { 0.0, 140.0, 1120.0, 700.0, 330.0, 0};
const double alpha[6] = { 0.0*PI/180,-90.0*PI/180,0.0*PI/180,0.0*PI/180,90.0*PI/180,90.0*PI/180 };

//电缸机械臂计算角度所需各项参数,结合图纸标注
const double firstarmParameter[1+5] = {0, 0.85024219, 0.26356388, 272.80945731, 652.53352404, 731.0};
const double secondarmParameter[1+5] = {0, 0.33114189, 0.52558379, 459.99049915, 346.8083044, 545.05523021};
const double rotaryarmParameter[1+10] = {0, 0.25255428, 0.19098292, 1.43996893, 640.31242374, 229.98156198, 525.07, 322.7234966, 150.0, 260.0, 191.63767897};

//电机初始角度值
const double initialAngle_1[]={0,75.0,62.0,318.7,299.3,38.1,347.4};//第一套电机的初始角度
//const double initialAngle_2[]={0,257.0,280.5,220.0,263.0,138.0,272.0};//第二套电机的初始角度
Electrickinematics::Electrickinematics()
{

}

void Electrickinematics::kinematics(double* theta_input, Eigen::Matrix4d *T06_output)//正运动学，用于检验
{
        Eigen::Matrix4d T[7];//两杆之间的变换矩阵,T[0]不用
        for (int i = 1; i <= 6; i++)
        {
                T[i](0, 0) = cos(theta_input[i]); //cos(x), x为弧度
                T[i](0, 1) = -sin(theta_input[i]);//sin(x), x为弧度
                T[i](0, 2) = 0;
                T[i](0, 3) = a[i-1];
                T[i](1, 0) = sin(theta_input[i]) * cos(alpha[i - 1]);
                T[i](1, 1) = cos(theta_input[i]) * cos(alpha[i - 1]);
                T[i](1, 2) = -sin(alpha[i - 1]);
                T[i](1, 3) = -sin(alpha[i - 1]) * d[i];
                T[i](2, 0) = sin(theta_input[i]) * sin(alpha[i - 1]);
                T[i](2, 1) = cos(theta_input[i]) * sin(alpha[i - 1]);
                T[i](2, 2) = cos(alpha[i - 1]);
                T[i](2, 3) = cos(alpha[i - 1]) * d[i];
                T[i](3, 0) = 0;
                T[i](3, 1) = 0;
                T[i](3, 2) = 0;
                T[i](3, 3) = 1;

        }
        Eigen::Matrix4d T06 = T[1]*T[2]*T[3]*T[4]*T[5]*T[6]; //T06描述了末端连杆坐标系{6}相对于基坐标系{0}的位姿
        *T06_output = T06;
}

bool Electrickinematics::inverse_kinematics(Eigen::Matrix4d *T06_input,double* theta_output)
{
    const double  theta1_min = -180.0*PI/180;
    const double  theta1_max = 180.0*PI/180;

    const double  theta2_min = -20.68*PI/180;
    const double  theta2_max = 20.76*PI/180;

    const double  theta3_min = 9.72*PI/180;
    const double  theta3_max = 47.17*PI/180;

    const double  theta4_min = -7.03*PI/180;
    const double  theta4_max = 49.02*PI/180;

    const double  theta5_min = 0.0*PI/180;
    const double  theta5_max = 180.0*PI/180;

    const double  theta6_min = -180.0*PI/180;
    const double  theta6_max = 180.0*PI/180;

    const double theta_min[6] = {theta1_min, theta2_min, theta3_min,
                                theta4_min, theta5_min, theta6_min};
    const double theta_max[6] = {theta1_max, theta2_max, theta3_max,
                                theta4_max, theta5_max, theta6_max};


    const double TOLERANCE = 0.001; //由于PI值为约数，三角函数计算也只是约数，定义此变量作为阈值

    Eigen::Matrix4d T06 = *T06_input;
    double nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz;
    nx = T06(0, 0); ox = T06(0, 1); ax = T06(0, 2); px = T06(0, 3);
    ny = T06(1, 0); oy = T06(1, 1); ay = T06(1, 2); py = T06(1, 3);
    nz = T06(2, 0); oz = T06(2, 1); az = T06(2, 2); pz = T06(2, 3);
    T06(3, 0) = 0;  T06(3, 1) = 0;  T06(3, 2) = 0;  T06(3, 3) = 1;


    //bool sucess = false; //是否结算处合适的逆解

    std::map<int, double> theta_select;

//calculate theta1;theta1 scope: -180°~180°;
    double theta1 = atan2(py-d[6]*ay, px-d[6]*ax);
    theta_output[1] = theta1;


//step1://calculate theta5;theta5 scope: 0°~180°;
    double theta5 = acos(ax*sin(theta1) - ay*cos(theta1));
    theta_output[5] = theta5;


//step2://calculate theta6;theta6 scope: -180°~180°;
    double theta6 = atan2(
                (ox*sin(theta1)-oy*cos(theta1)),
                (ny*cos(theta1)-nx*sin(theta1))
                );
    theta_output[6] = theta6;


//step3: //calculate theta234;theta234 scope: 17.99°~116.95°;

       double theta234 = atan2(-az , (ax*cos(theta1) + ay*sin(theta1)));

//step4: //calculate theta3;theta3 scope: 9.72°~47.17°;

       double P = -px*cos(theta1) - py*sin(theta1) + a[1] + (a[4]+d[6]*sin(theta5))*cos(theta234) + d[5]*sin(theta234) ;
       double Q = -pz + d[1]  + d[5]*cos(theta234) -(a[4]+d[6]*sin(theta5))*sin(theta234);
       double theta3 = acos((P*P+Q*Q-a[2]*a[2]-a[3]*a[3])/(2*a[3]*a[2]));
       theta_output[3] = theta3;


//step5://calculate theta23;theta23 scope: -10.96°~67.93°;
       double delta =  atan2(Q, P);
       double theta23;
       double theta23_initial = asin(a[2]*sin(theta3)/sqrt(P*P + Q*Q))-delta;

       if( theta23_initial>=-11*PI/180   && theta23_initial <= 68*PI/180)
           theta23=theta23_initial;
       else if( theta23_initial>=-11*PI/180+PI   && theta23_initial <= 68*PI/180+PI)
           theta23=theta23_initial-PI;
       else
           theta23=theta23_initial+PI;

//step6://calculate theta2;theta2 scope: -20.68°~20.76°;

       theta_output[2] = theta23-theta3;


//step7://calculate theta4;theta4 scope: -7.03°~49.02°

       theta_output[4] = theta234-theta23;


//step8: //判断各个关节角度的范围是否符合要求

       // 将角度与关节极限位置比较，避免出现 -1.5e-05 这种约等于0的数，对后面的关节角范围判断产生干扰
           if( abs( theta_output[1] ) <= TOLERANCE/10 )
                 theta_output[1] =0;

           if( abs( theta_output[2] ) <= TOLERANCE/10 )
                 theta_output[2] =0;

           if( abs( theta_output[3] ) <= TOLERANCE/10 )
                 theta_output[3] =0;

           if( abs( theta_output[4] ) <= TOLERANCE/10 )
                 theta_output[4] =0;

           if( abs( theta_output[5] ) <= TOLERANCE/10 )
                 theta_output[5] =0;

           if( abs( theta_output[6] ) <= TOLERANCE/10 )
                 theta_output[6] =0;

           if(
                   theta_output[1]    <=  theta1_max   && theta_output[1] >= theta1_min
                   && theta_output[2] <=  theta2_max   && theta_output[2] >= theta2_min
                   && theta_output[3] <=  theta3_max   && theta_output[3] >= theta3_min
                   && theta_output[4] <=  theta4_max   && theta_output[4] >= theta4_min
                   && theta_output[5] <=  theta5_max   && theta_output[5] >= theta5_min
                   && theta_output[6] <=  theta6_max   && theta_output[6] >= theta6_min

              )
           {
               theta_output[1]=angle(theta_output[1]);
               theta_output[2]=angle(theta_output[2]);
               theta_output[3]=angle(theta_output[3]);
               theta_output[4]=angle(theta_output[4]);
               theta_output[5]=angle(theta_output[5]);
               theta_output[6]=angle(theta_output[6]);
               cout<<"theta[1]="<<theta_output[1]<<endl;
               cout<<"theta[2]="<<theta_output[2]<<endl;
               cout<<"theta[3]="<<theta_output[3]<<endl;
               cout<<"theta[4]="<<theta_output[4]<<endl;
               cout<<"theta[5]="<<theta_output[5]<<endl;
               cout<<"theta[6]="<<theta_output[6]<<endl;
               cout<<"\n"<<endl;

               return true;

           }
           else return false;

}


double Electrickinematics::armEndDistanceCalculate(double *theta_input, Eigen::Matrix4d T06_input)
{
    Eigen::Matrix4d T06_output;
    kinematics(theta_input, &T06_output);
    return  ( sqrt(  (T06_output(0,3)- T06_input(0,3))*(T06_output(0,3)- T06_input(0,3))
                   + (T06_output(1,3)- T06_input(1,3))*(T06_output(1,3)- T06_input(1,3))
                   + (T06_output(2,3)- T06_input(2,3))*(T06_output(2,3)- T06_input(2,3))
                  )
            );
}

void Electrickinematics::displacementToAngle(double *parameter_input, double *theta_input)
{
    theta_input[0] = 0;
     //theta1
    theta_input[1] = parameter_input[1];

    //first arm displacement to theta2
    double theta_fisrtarm = acos((firstarmParameter[3]*firstarmParameter[3] + firstarmParameter[4]*firstarmParameter[4] -
                          (firstarmParameter[5] + parameter_input[2])*(firstarmParameter[5] + parameter_input[2]))
                         /(2*firstarmParameter[3]*firstarmParameter[4]));
    theta_input[2] = firstarmParameter[1] + firstarmParameter[2] + theta_fisrtarm - PI;

    //second arm displacement to theta3
    double theta_secondarm = acos((secondarmParameter[3]*secondarmParameter[3] + secondarmParameter[4]*secondarmParameter[4] -
                          (secondarmParameter[5] + parameter_input[3])*(secondarmParameter[5] + parameter_input[3]))
                         /(2*secondarmParameter[3]*secondarmParameter[4]));
    theta_input[3] = PI - secondarmParameter[1] - secondarmParameter[2] - theta_secondarm;

    //rotary arm displacement to theta4
    double theta_CBD = acos((rotaryarmParameter[4]*rotaryarmParameter[4] + rotaryarmParameter[5]*rotaryarmParameter[5] -
                          (rotaryarmParameter[6] + parameter_input[4])*(rotaryarmParameter[6] + parameter_input[4]))
                         /(2*rotaryarmParameter[4]*rotaryarmParameter[5]));
    double theta_EBF = PI - rotaryarmParameter[1] - rotaryarmParameter[2] - theta_CBD;
    double line_EF = sqrt(rotaryarmParameter[7]*rotaryarmParameter[7] + rotaryarmParameter[8]*rotaryarmParameter[8]
                         - 2*rotaryarmParameter[7]*rotaryarmParameter[8]*cos(theta_EBF));
    double theta_BFE = acos((rotaryarmParameter[8]*rotaryarmParameter[8] + line_EF*line_EF
                          - rotaryarmParameter[7]*rotaryarmParameter[7])
                           /(2*rotaryarmParameter[8]*line_EF));
    double theta_EFG = acos((line_EF*line_EF + rotaryarmParameter[10]*rotaryarmParameter[10]
                           - rotaryarmParameter[9]*rotaryarmParameter[9])
                            /(2*line_EF*rotaryarmParameter[10]));
    theta_input[4] = rotaryarmParameter[3] + theta_BFE + theta_EFG - PI;

    //swing displacement to theta5
    theta_input[5] = parameter_input[5];

    //end swing displacement to theta6
    theta_input[6] = parameter_input[6];

    for(int i=1;i<=6;i++)
    {
        std::cout<<"displacementToAngle theta_["<<i<<"]="<<angle(theta_input[i])<<endl;

    }
}

void Electrickinematics::angleToDisplacement(double *theta_output, double *paramater_output)
{

    paramater_output[0] = 0;
     //theta1
    paramater_output[1] = theta_output[1];

    //theta2 to first arm displacement
    double theta_fisrtarm = PI - firstarmParameter[1] - firstarmParameter[2] + rad(theta_output[2]);
    paramater_output[2] = sqrt(firstarmParameter[3]*firstarmParameter[3] + firstarmParameter[4]*firstarmParameter[4]
                             - 2*firstarmParameter[3]*firstarmParameter[4]*cos(theta_fisrtarm)) - firstarmParameter[5];

    //theta3 to second arm displacement
    double theta_secondarm = PI - secondarmParameter[1] - secondarmParameter[2] - rad(theta_output[3]);
    paramater_output[3] = sqrt(secondarmParameter[3]*secondarmParameter[3] + secondarmParameter[4]*secondarmParameter[4]
            - 2*secondarmParameter[3]*secondarmParameter[4]*cos(theta_secondarm)) - secondarmParameter[5];

    //theta4 to rotary arm displacement
    double theta_BFG = PI - rotaryarmParameter[3] + rad(theta_output[4]);
    double line_BG = sqrt(rotaryarmParameter[8]*rotaryarmParameter[8] + rotaryarmParameter[10]*rotaryarmParameter[10]
            - 2*rotaryarmParameter[8]*rotaryarmParameter[10]*cos(theta_BFG));
    double theta_EBG = acos((rotaryarmParameter[7]*rotaryarmParameter[7] + line_BG*line_BG
            - rotaryarmParameter[9]*rotaryarmParameter[9])
             /(2*rotaryarmParameter[7]*line_BG));
    double theta_FBG = acos((rotaryarmParameter[8]*rotaryarmParameter[8] + line_BG*line_BG
            - rotaryarmParameter[10]*rotaryarmParameter[10])
             /(2*rotaryarmParameter[8]*line_BG));
    double theta_CBD = PI - rotaryarmParameter[1] - rotaryarmParameter[2] - theta_EBG - theta_FBG;
    paramater_output[4] = sqrt(rotaryarmParameter[4]*rotaryarmParameter[4] + rotaryarmParameter[5]*rotaryarmParameter[5]
            - 2*rotaryarmParameter[4]*rotaryarmParameter[5]*cos(theta_CBD)) - rotaryarmParameter[6];

    //theta5 to swing displacement
    paramater_output[5] = theta_output[5];
    //theta6 to end swing displacement
    paramater_output[6] = theta_output[6];

    for(int i=1;i<=6;i++)
    {
        std::cout<<"angleToDisplacement["<<i<<"]="<<paramater_output[i]<<endl;
    }
}


/**
 * @brief hydraulicarmkinematicsdynamics::kinematics2 根据相对基坐标系的XYZ轴平移，绕X Y Z旋转角度（X-侧摆，Y-俯仰，Z-横滚），计算机械臂末端位姿。此函数用于手柄对机械臂的控制
 * @param XPos_input-相对于基坐标系X轴的位移
 * @param YPos_input-相对于基坐标系Y轴的位移
 * @param ZPos_input-相对于基坐标系Z轴的位移
 * @param Xtheta_input-绕基坐标系X轴的角度，侧摆角
 * @param Ytheta_input-绕基坐标系Y轴的角度，俯仰角
 * @param Ztheta_input-绕基坐标系Z轴的角度，横滚角
 * @param T06_output-输出矩阵，机械臂末端位姿
 */
void Electrickinematics::kinematics2(double* XPos_input, double* YPos_input, double* ZPos_input, double* XRotate_input, double* YRotate_input, double* ZRotate_input, Eigen::Matrix4d *T06_output)
{
    double nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz;

    *XRotate_input=rad(*ZRotate_input);
    *YRotate_input=rad(*YRotate_input);
    *ZRotate_input=rad(*XRotate_input);


    nx = cos( *ZRotate_input ) * cos( *YRotate_input );
    ny = sin( *ZRotate_input ) * cos( *XRotate_input ) +  cos( *ZRotate_input ) * sin( *YRotate_input ) *sin( *XRotate_input );
    nz = sin( *ZRotate_input ) * sin( *XRotate_input ) -  cos( *ZRotate_input ) * sin( *YRotate_input ) *cos( *XRotate_input );

    ox =  - cos( *YRotate_input ) * sin( *ZRotate_input ) ;
    oy = cos( *ZRotate_input ) * cos( *XRotate_input ) -  sin( *ZRotate_input ) * sin( *YRotate_input ) *sin( *XRotate_input );
    oz = cos( *ZRotate_input ) * sin( *XRotate_input ) +  sin( *ZRotate_input ) * sin( *YRotate_input ) *cos( *XRotate_input );

    ax = sin(*YRotate_input );
    ay = - sin( *XRotate_input ) * cos( *YRotate_input ) ;
    az = cos(*YRotate_input)*cos(*XRotate_input) ;





    const double TOLERANCE = 0.001;
    if( abs( nx ) <= TOLERANCE/10 )
          nx =0;
    if( abs( ny ) <= TOLERANCE/10 )
          ny =0;
    if( abs( nz ) <= TOLERANCE/10 )
          nz =0;
    if( abs( ox ) <= TOLERANCE/10 )
          ox =0;
    if( abs( oy ) <= TOLERANCE/10 )
          oy =0;
    if( abs( oz ) <= TOLERANCE/10 )
          oz =0;
    if( abs( ax ) <= TOLERANCE/10 )
          ax =0;
    if( abs( ay ) <= TOLERANCE/10 )
          ay =0;
    if( abs( az ) <= TOLERANCE/10 )
          az =0;

    px = *XPos_input;
    py = *YPos_input;
    pz = *ZPos_input;

    (*T06_output)(0, 0) = nx; (*T06_output)(0, 1) = ox; (*T06_output)(0, 2) = ax; (*T06_output)(0, 3) = px;
    (*T06_output)(1, 0) = ny; (*T06_output)(1, 1) = oy; (*T06_output)(1, 2) = ay; (*T06_output)(1, 3) = py;
    (*T06_output)(2, 0) = nz; (*T06_output)(2, 1) = oz; (*T06_output)(2, 2) = az; (*T06_output)(2, 3) = pz;
    (*T06_output)(3, 0) = 0;  (*T06_output)(3, 1) = 0;  (*T06_output)(3, 2) = 0;  (*T06_output)(3, 3) = 1;


}


/**
 * @brief Electrickinematics::electricmachineryToDisplacement
 * @param electricmachinery_input 考虑减速器减速比。没用到
 * @param parameter_input
 */
void Electrickinematics::electricmachineryToDisplacement(double *electricmachinery_input, double *parameter_input)
{
    parameter_input[0]=0;
    parameter_input[1]=(electricmachinery_input[1]-initialAngle_1[1])/(140.0/50.0)/(106.0/20.0);//底座回转电机，140/50为减速机减速比,106/20为齿轮减速比
    parameter_input[2]=(electricmachinery_input[2]-initialAngle_1[2])/(360.0/10.0)/7.0;//大臂电缸，360.0°/10.0mm为丝杠导程，7为减速机减速比
    parameter_input[3]=(electricmachinery_input[3]-initialAngle_1[3])/(360.0/5.0)/2.0;//二臂电缸，360.0°/10.0mm为丝杠导程，2为减速机减速比
    parameter_input[4]=(electricmachinery_input[4]-initialAngle_1[4])/(360.0/5.0)/2.0;//转动电缸，360.0°/10.0mm为丝杠导程，2为减速机减速比
    parameter_input[5]=(electricmachinery_input[5]-initialAngle_1[5])/(90.0/50.0)/(85.0/20.0);//摆动电机，90/50为减速机减速比,85/20为齿轮减速比
    parameter_input[6]=(electricmachinery_input[6]-initialAngle_1[6])/(90.0/50.0)/(85.0/20.0);//末端回转电机，90/50为减速机减速比,85/20为齿轮减速比
}

/**
 * @brief Electrickinematics::displacementToElectricmachinery 考虑减速器和减速齿轮，电缸位移到电机转角
 * @param parameter_output
 * @param electricmachinery_output
 */
void Electrickinematics::displacementToElectricmachinery(double *parameter_output, double *electricmachinery_output)
{
    electricmachinery_output[0]=0;
    electricmachinery_output[1]=parameter_output[1]*(140.0/50.0)*(106.0/20.0)+initialAngle_1[1];
    electricmachinery_output[2]=parameter_output[2]*(360.0/10.0)*7.0+initialAngle_1[2];
    electricmachinery_output[3]=parameter_output[3]*(360.0/5.0)*2.0+initialAngle_1[3];
    electricmachinery_output[4]=parameter_output[4]*(360.0/5.0)*2.0+initialAngle_1[4];
    electricmachinery_output[5]=parameter_output[5]*(90.0/50.0)*(85.0/20.0)+initialAngle_1[5];
    electricmachinery_output[6]=parameter_output[6]*(90.0/50.0)*(85.0/20.0)+initialAngle_1[6];
}
