#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "DBUS.h"
#include "MY_define.h"
#include "Motors.h"
#include "controller.h"
#include "Power_Ctrl.h"

//舵轮底盘参数
typedef struct
{
    float m;//质量
    float J;//转动惯量
    float R;//舵轮中心点到底盘中心点的距离
    float r;//舵轮半径
}ChassisParams;

//舵轮轮电机状态参数
typedef struct
{
    float Pos_angle;//舵轮方位角
    float Frd_angle;//舵向角的观测值
    float speed;//电机输出轴转速
    float tau_need;//需要的控制力矩
    uint8_t forward_3508;

    PID_t pid;
}SwerveWheelState;

//底盘运动状态参数
typedef struct
{
    float Vx_extern;//x轴观测速度
    float Vy_extern;//y轴观测速度
    float Vw_extern;//旋转的观测速度

    float Vx_dot_need;//x轴需要的加速度
    float Vy_dot_need;//y轴需要的加速度
    float Vw_dot_need;//旋转需要的加速度

    PID_t Chassis_pid_x;//专门为底盘观测速度x使用的pid
    PID_t Chassis_pid_y;
    PID_t Chassis_pid_w;
}ChassisState;

//舵轮舵电机参数
typedef struct
{
    float zero_offset;//0点偏移量
    float Frd_angle;//舵向角的观测值
    float tau_need;
    PID_t Swerve_pid_speed;//舵电机使用的pid
    PID_t Swerve_pid_angle;
    float speed;//舵电机输出轴转速
    float last_angle_target;
}SwerveState;


void chassis_init(ChassisParams *params, SwerveWheelState wheelStates[4], ChassisState *state, SwerveState Swerve[4]);
float encode_to_rad(int zero_offset, int raw_enconde);
void remote_control(DBUS_Typedef DBUS, ChassisState state, ChassisParams *params, MOTOR_Typdef *Motor, SwerveWheelState wheelStates[4], SwerveState Swerve[4]);
void cal_wheel_torque(ChassisParams *params, SwerveWheelState wheelStates[4], ChassisState state);
void stop(DBUS_Typedef *DBUS,SwerveState Swerve[4],SwerveWheelState wheel[4]);
void cal_Swerve_torque(MOTOR_Typdef *Motor, SwerveWheelState wheelStates[4], ChassisParams *params, ChassisState state, SwerveState Swerve[4]);
#endif
