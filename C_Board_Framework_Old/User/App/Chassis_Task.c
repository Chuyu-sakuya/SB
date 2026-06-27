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
/*
 *@attention
 *1.在关于速度的变量中，带有0的变量指的是观测的速度，不带有的则是目标控制速度
 *
 *
 *
 *
 */
#define ENCODER_6020 8192.0
#define PI 3.14159265358979323846
void chassis_init(ChassisParams *params, SwerveWheelState *wheelStates[4], ChassisState *state,SwerveState *Swerve)
{
    params->m=20.0;
    params->J=1.0;
    params->R=0.3;
    params->r=0.05;


    Swerve->zero_offset[0]=1000;
    Swerve->zero_offset[1]=1000;
    Swerve->zero_offset[2]=1000;
    Swerve->zero_offset[3]=1000;


}

double encode_to_rad(int zero_offset,int raw_enconde)
{
    int diff=raw_enconde-zero_offset;
    if (diff>4096){diff-=8192;};
    if (diff<-4096){diff+=8192;};

    double Frd_angle=((double)diff/ENCODER_6020)*(2.0*PI);
    return Frd_angle;

}



void remote_control(DBUS_Typedef *DBUS,ChassisState *state,ChassisParams *params,MOTOR_Typdef *Motor,SwerveWheelState *wheelStates[4])
{

    double Vw_1_rad=(Motor->DJI_3508_Chassis_0.DATA.Speed_now/19)*(2.0*PI/60.0);
    double Vw_2_rad=(Motor->DJI_3508_Chassis_1.DATA.Speed_now/19)*(2.0*PI/60.0);
    double Vw_3_rad=(Motor->DJI_3508_Chassis_2.DATA.Speed_now/19)*(2.0*PI/60.0);
    double Vw_4_rad=(Motor->DJI_3508_Chassis_3.DATA.Speed_now/19)*(2.0*PI/60.0);

    state->Vx_extern = (params->r/4.0) * (Vw_1_rad * cos(wheelStates[0]->Frd_angle) + Vw_4_rad * cos(wheelStates[3]->Frd_angle) + Vw_2_rad * cos(wheelStates[1]->Frd_angle) + Vw_3_rad * cos(wheelStates[2]->Frd_angle));
    state->Vy_extern = (params->r/4.0) * (Vw_1_rad * sin(wheelStates[0]->Frd_angle) + Vw_4_rad * sin(wheelStates[3]->Frd_angle) + Vw_2_rad * sin(wheelStates[1]->Frd_angle) + Vw_3_rad * sin(wheelStates[2]->Frd_angle));
    state->Vw_extern = (-params->r/(4.0*params->R)) * (-Vw_1_rad * sin(wheelStates[0]->Frd_angle) + Vw_2_rad * cos(wheelStates[1]->Frd_angle) + Vw_3_rad * sin(wheelStates[2]->Frd_angle) - Vw_4_rad * cos(wheelStates[3]->Frd_angle));


    double CH2=DBUS->Remote.CH2/660.0*5.0;
    double CH3=DBUS->Remote.CH3/660.0*5.0;
    double dial=DBUS->Remote.Dial/660.0*5.0;

    state->Vx_dot_need=PID_Calculate(&state->Chassis_pid_x,state->Vx_extern,CH2);
    state->Vy_dot_need=PID_Calculate(&state->Chassis_pid_y,state->Vy_extern,CH3);
    state->Vw_dot_need=PID_Calculate(&state->Chassis_pid_w,state->Vw_extern,dial);




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

        double x_dot_0=state->Vx_extern;
        double y_dot_0=state->Vy_extern;
        double w_dot_0=state->Vw_extern;

        double x_ddot_0=state->Vx_dot_need;
        double y_ddot_0=state->Vy_dot_need;
        double w_ddot_0=state->Vw_dot_need;


        //计算前馈力矩
        double ff_x=m*x_ddot_0*cos(Frd_angle)*R;
        double ff_y=m*y_ddot_0*sin(Frd_angle)*R;
        double ff_w=J*w_ddot_0*sin(Frd_angle-Pos_angle)*R;

        double tau_forward=(ff_x+ff_y+ff_w)*r/(4*R);

        //计算pid控制器
        double P_x=x_dot_0*cos(Frd_angle);
        double P_y=y_dot_0*sin(Frd_angle);
        double P_w=R*w_dot_0*sin(Frd_angle-Pos_angle);

        double target=(P_x+P_y+P_w)/r;

        double tau_pid=PID_Calculate(&wheelStates[i]->pid, wheelStates[i]->speed, target);

        wheelStates[i]->tau_need=(tau_forward+tau_pid)*1600;
    }


}

void cal_Swerve_torque(MOTOR_Typdef *Motor,SwerveWheelState *wheelStates[4], ChassisParams *params,ChassisState *state,SwerveState *Swerve[4])
{

    double tau_need=0;
    for (int i=0; i<4; i++)
    {

        double x_dot_0=state->Vx_extern;
        double y_dot_0=state->Vy_extern;
        double w_dot_0=state->Vw_extern;

        double x_ddot_0=state->Vx_dot_need;
        double y_ddot_0=state->Vy_dot_need;
        double w_ddot_0=state->Vw_dot_need;

        double Vx_i=x_dot_0-params->R*w_dot_0*sin(wheelStates[i]->Pos_angle);
        double Vy_i=y_dot_0+params->R*w_dot_0*cos(wheelStates[i]->Pos_angle);
        double V_i=sqrt(Vx_i*Vx_i+Vy_i*Vy_i);//速度向量的大小

        double A_xi=x_ddot_0-params->R*(w_ddot_0*sin(wheelStates[i]->Pos_angle)+w_dot_0*w_dot_0*cos(wheelStates[i]->Pos_angle));
        double A_yi=y_ddot_0+params->R*(w_ddot_0*cos(wheelStates[i]->Pos_angle)-w_dot_0*w_dot_0*sin(wheelStates[i]->Pos_angle));
        double A_i=sqrt(A_xi*A_xi+A_yi*A_yi);//加速度向量的大小

        if (V_i>0.0001)
        {
            double target_angle=atan2(Vy_i,Vx_i);

            double err=target_angle-wheelStates[i]->Frd_angle;

            if (err>PI)
            {
                err-=2*PI;
            }
            if (err<-PI)
            {
                err+=2*PI;
            }

            double part1 = state->Vx_extern * state->Vy_dot_need
              - state->Vy_extern * state->Vx_dot_need
              - state->Vw_extern * (state->Vx_extern * state->Vx_extern + state->Vy_extern * state->Vy_extern);

            double part2 = params->R * cos(wheelStates[i]->Pos_angle)
              * (state->Vw_dot_need * state->Vx_extern
              - state->Vw_extern * (state->Vx_dot_need + state->Vw_extern * state->Vy_extern));

            double part3 = params->R * sin(wheelStates[i]->Pos_angle)
                         * (state->Vw_dot_need * state->Vy_extern
                         - state->Vw_extern * (state->Vy_dot_need + state->Vw_extern * state->Vx_extern));

            double w_dot_ff=(part1+part2+part3)/(V_i*V_i);

            double target_speed=w_dot_ff+PID_Calculate(&Swerve[i]->Swerve_pid_speed, 0, err);

         tau_need=PID_Calculate(&Swerve[i]->Swerve_pid_angle,wheelStates[i]->speed , target_speed);



        }
        else if (A_i>0.0001)
        {
            double target_angle=atan2(A_yi,A_xi);

            double err=target_angle-wheelStates[i]->Frd_angle;

            double target_speed=0.0+PID_Calculate(&Swerve[i]->Swerve_pid_speed, 0, err);

            tau_need=PID_Calculate(&Swerve[i]->Swerve_pid_angle,wheelStates[i]->speed , target_speed);


        }
        else
        {
          tau_need=0;

        }
            Swerve[i]->tau_need=tau_need*13000;
    }

}

