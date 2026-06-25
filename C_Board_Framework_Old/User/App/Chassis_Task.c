#include <Chassis_Task.h>
#include <math.h>


/*************************************************************************************************************
                                 +
                          0     |     1
                                |
                                |
                                |
                                |
                  ------------------------------+
                                |
                                |
                                |
                                |
                         2      |      3


*************************************************************************************************************/

void chassis_init(ChassisParams *params, SwerveWheelState *wheelStates[4], ChassisState *state)
{
    params->m=20.0;
    params->J=1.0;
    params->R=0.3;
    params->r=0.05;

    for (int i=0; i<4; i++)
    {
        wheelStates[i]->Pos_angle=0.0;
        wheelStates[i]->Frd_angle=0.0;
    }
}

void cal_wheel_torque(ChassisParams *params, SwerveWheelState *wheelStates[4], ChassisState *state)
{

    double R=params->R;
    double r=params->r;
    double m=params->m;
    double J=params->J;

    for (int i=0; i<4; i++)
    {
        double Pos_angle=wheelStates[i]->Pos_angle;
        double Frd_angle=wheelStates[i]->Frd_angle;


        //计算前馈力矩
        double ff_x=m*state->Vx_dot_need*cos(Frd_angle);
        double ff_y=m*state->Vy_dot_need*sin(Frd_angle);
        double ff_w=J*state->Vw_dot_need*sin(Frd_angle-Pos_angle);

        double tau_forward=(ff_x+ff_y+ff_w)*r/(4*R);

        //计算pid控制器
        double P_x=state->Vx_extern*cos(Frd_angle);
        double P_y=state->Vy_extern*sin(Frd_angle);
        double P_w=state->Vw_extern*sin(Frd_angle-Pos_angle);

        double target=(P_x+P_y+P_w)/r;

        double tau_pid=PID_Calculate(&wheelStates[i]->pid, wheelStates[i]->speed, target);

        wheelStates[i]->tau_need=tau_forward+tau_pid;
    }





}