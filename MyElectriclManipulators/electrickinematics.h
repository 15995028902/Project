#ifndef ELECTRICKINEMATICS_H
#define ELECTRICKINEMATICS_H

#include <iostream>
#include <math.h>

#include <Eigen/Eigen>
using namespace Eigen;

using namespace ::std;

#define PI acos(-1.0)
#define rad(x)  (x*PI/180)  //角度转化为弧度
#define angle(x) (x*180/PI) //弧度转化为角度

class Electrickinematics
{
public:
    Electrickinematics();
    void kinematics(double *theta_input, Eigen::Matrix4d *T06_output);//正运动学求解，输入关节角度，输出T06矩阵
    bool inverse_kinematics(Eigen::Matrix4d *T06_input,  double* theta_output);//逆运动学求解，输入T06矩阵，输出关节角度；
    void electricmachineryToDisplacement(double *electricmachinery_input,double *parameter_input);//电机绝对角度（角度制）到电缸或电机位移输出的转换;
    void displacementToAngle(double *parameter_input, double *theta_input);//电缸或电机位移输出到关节角度的转换；
    void angleToDisplacement(double *theta_output, double* paramater_output);//关节角度到电缸或电机位移输出的转换；
    void displacementToElectricmachinery(double *parameter_output,double *electricmachinery_output);//电缸或电机位移输出到电机绝对角度（角度制）的转换;
    double armEndDistanceCalculate(double *theta_input, Eigen::Matrix4d T06_input);
    void kinematics2(double *XPos_input, double *YPos_input, double *ZPos_input, double *XRotate_input, double *YRotate_input, double *ZRotate_input, Eigen::Matrix4d *T06_output);
};

#endif // ELECTRICKINEMATICS_H



