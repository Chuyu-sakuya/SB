#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "DBUS.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "Power_Ctrl.h"
void swerveWheelResolve(double *Wheel_Rpm,float Vx_truth,float Vy_truth,float Vw_truth);
#endif
