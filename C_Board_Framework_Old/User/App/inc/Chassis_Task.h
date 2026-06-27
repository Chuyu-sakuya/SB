#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "DBUS.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "All_Init.h"
#include "Power_Ctrl.h"

//舵轮底盘参数
typedef struct
{
    double m;//质量
    double J;//转动惯量
    double R;//舵轮中心点到底盘中心点的距离
    double r;//舵轮半径
}ChassisParams;

//舵轮轮电机状态参数
typedef struct
{
    double Pos_angle;//舵轮方位角
    double Frd_angle;//舵向角的观测值
    double speed;//电机输出轴转速
    double tau_need;//需要的控制力矩

    PID_t pid;
}SwerveWheelState;

//底盘运动状态参数
typedef struct
{
    double Vx_extern;//x轴观测速度
    double Vy_extern;//y轴观测速度
    double Vw_extern;//旋转的观测速度

    double Vx_dot_need;//x轴需要的加速度
    double Vy_dot_need;//y轴需要的加速度
    double Vw_dot_need;//旋转需要的加速度

    PID_t Chassis_pid_x;//专门为底盘观测速度x使用的pid
    PID_t Chassis_pid_y;
    PID_t Chassis_pid_w;
}ChassisState;

//舵轮舵电机参数
typedef struct
{
    double zero_offset[4];//0点偏移量
    double Frd_angle;//舵向角的观测值
    double tau_need;
    PID_t Swerve_pid_speed;//舵电机使用的pid
    PID_t Swerve_pid_angle;
}SwerveState;

void swerveWheelResolve(double *Wheel_Rpm,float Vx_truth,float Vy_truth,float Vw_truth);
#endif
