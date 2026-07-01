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
float target_speed=0;
float target_angle=0;
float x_dot_0=0;
float y_dot_0=0;
float w_dot_0=0;

float x_ddot_0=0;
float y_ddot_0=0;
float w_ddot_0=0;

float Vx_i=0;
float Vy_i=0;
float V_i=0;

float A_xi=0;
float A_yi=0;
float A_i=0;
void chassis_init(ChassisParams *params, SwerveWheelState wheelStates[4], ChassisState *state,SwerveState Swerve[4])
{
    params->m=20.0;
    params->J=1.0;
    params->R=0.3;
    params->r=0.05;

    wheelStates[0].forward_3508=1;
    wheelStates[1].forward_3508=1;
    wheelStates[2].forward_3508=1;
    wheelStates[3].forward_3508=1;

    wheelStates[0].Pos_angle=0;
    wheelStates[1].Pos_angle=PI/2;
    wheelStates[2].Pos_angle=PI;
    wheelStates[3].Pos_angle=(PI*3)/2;

    float kpid_wheel_0[3]={10,0.01,0};
    float kpid_wheel_1[3]={10,0.01,0};
    float kpid_wheel_2[3]={10,0.01,0};
    float kpid_wheel_3[3]={10,0.01,0};

    float kpid_Swerve_angle_0[3]={8,0.01,0};
    float kpid_Swerve_angle_1[3]={8,0.01,0};
    float kpid_Swerve_angle_2[3]={8,0.01,0};
    float kpid_Swerve_angle_3[3]={8,0.01,0};

    float kpid_Swerve_speed_0[3]={0.175,0.001,0};
    float kpid_Swerve_speed_1[3]={0.175,0.001,0};
    float kpid_Swerve_speed_2[3]={0.175,0.001,0};
    float kpid_Swerve_speed_3[3]={0.175,0.001,0};

    float kpid_Chassis_x[3]={5,0,0};
    float kpid_Chassis_y[3]={5,0,0};
    float kpid_Chassis_w[3]={5,0,0};


    Swerve[0].zero_offset=1350;
    Swerve[1].zero_offset=1350;
    Swerve[2].zero_offset=1350+4096;
    Swerve[3].zero_offset=1350+4096;

    PID_Init(&wheelStates[0].pid, 10.0, 2.0,kpid_wheel_0, 0, 0,0,0,0,Integral_Limit);
    PID_Init(&wheelStates[1].pid, 10.0, 2.0, kpid_wheel_1, 0, 0,0,0,0,Integral_Limit);
    PID_Init(&wheelStates[2].pid, 10.0, 2.0, kpid_wheel_2, 0, 0,0,0,0,Integral_Limit);
    PID_Init(&wheelStates[3].pid, 10.0, 2.0, kpid_wheel_3, 0, 0,0,0,0,Integral_Limit);

    PID_Init(&Swerve[0].Swerve_pid_angle, 30.0, 5.0, kpid_Swerve_angle_0, 0, 0,0,0,0,Integral_Limit);
    PID_Init(&Swerve[1].Swerve_pid_angle, 30.0, 5.0, kpid_Swerve_angle_1, 0, 0,0,0,0,Integral_Limit);
    PID_Init(&Swerve[2].Swerve_pid_angle, 30.0, 5.0, kpid_Swerve_angle_2, 0, 0,0,0,0,Integral_Limit);
    PID_Init(&Swerve[3].Swerve_pid_angle, 30.0, 5.0, kpid_Swerve_angle_3, 0, 0,0,0,0,Integral_Limit);

    PID_Init(&Swerve[0].Swerve_pid_speed, 1.2, 0.3, kpid_Swerve_speed_0, 0, 0,0,0,0,Integral_Limit);
    PID_Init(&Swerve[1].Swerve_pid_speed, 1.2, 0.3, kpid_Swerve_speed_1, 0, 0,0,0,0,Integral_Limit);
    PID_Init(&Swerve[2].Swerve_pid_speed, 1.2, 0.3, kpid_Swerve_speed_2, 0, 0,0,0,0,Integral_Limit);
    PID_Init(&Swerve[3].Swerve_pid_speed, 1.2, 0.3, kpid_Swerve_speed_3, 0, 0,0,0,0,Integral_Limit);

    PID_Init(&state->Chassis_pid_x, 16384, 2.0, kpid_Chassis_x, 0, 0,0,0,0,Integral_Limit);
    PID_Init(&state->Chassis_pid_y, 16384, 2.0, kpid_Chassis_y, 0, 0,0,0,0,Integral_Limit);
    PID_Init(&state->Chassis_pid_w, 16384, 2.0, kpid_Chassis_w, 0, 0,0,0,0,Integral_Limit);
}

float encode_to_rad(int zero_offset,int raw_enconde)
{
    int diff=raw_enconde-zero_offset;
    if (diff>4096){diff-=8192;};
    if (diff<-4096){diff+=8192;};

    float  Frd_angle=((double)diff/ENCODER_6020)*(2.0*PI);
    return Frd_angle;

}



void remote_control(DBUS_Typedef DBUS,ChassisState state,ChassisParams *params,MOTOR_Typdef *Motor,SwerveWheelState wheelStates[4],SwerveState Swerve[4])
{

    float Vw_1_rad=(Motor->DJI_3508_Chassis_0.DATA.Speed_now/19.0)*(2.0*PI/60.0);
    float Vw_2_rad=(Motor->DJI_3508_Chassis_1.DATA.Speed_now/19.0)*(2.0*PI/60.0);
    float Vw_3_rad=(Motor->DJI_3508_Chassis_2.DATA.Speed_now/19.0)*(2.0*PI/60.0);
    float Vw_4_rad=(Motor->DJI_3508_Chassis_3.DATA.Speed_now/19.0)*(2.0*PI/60.0);

    wheelStates[0].speed=Motor->DJI_3508_Chassis_0.DATA.Speed_now/19.0*(2.0*PI/60.0);
    wheelStates[1].speed=Motor->DJI_3508_Chassis_1.DATA.Speed_now/19.0*(2.0*PI/60.0);
    wheelStates[2].speed=Motor->DJI_3508_Chassis_2.DATA.Speed_now/19.0*(2.0*PI/60.0);
    wheelStates[3].speed=Motor->DJI_3508_Chassis_3.DATA.Speed_now/19.0*(2.0*PI/60.0);

    Swerve[0].speed=Motor->DJI_6020_Swerve_0.DATA.Speed_now*(2.0*PI/60.0);
    Swerve[1].speed=Motor->DJI_6020_Swerve_1.DATA.Speed_now*(2.0*PI/60.0);
    Swerve[2].speed=Motor->DJI_6020_Swerve_2.DATA.Speed_now*(2.0*PI/60.0);
    Swerve[3].speed=Motor->DJI_6020_Swerve_3.DATA.Speed_now*(2.0*PI/60.0);

    state.Vx_extern = (params->r/4.0) * (Vw_1_rad * cosf(wheelStates[0].Frd_angle) + Vw_4_rad * cosf(wheelStates[3].Frd_angle) + Vw_2_rad * cosf(wheelStates[1].Frd_angle) + Vw_3_rad * cosf(wheelStates[2].Frd_angle));
    state.Vy_extern = (params->r/4.0) * (Vw_1_rad * sinf(wheelStates[0].Frd_angle) + Vw_4_rad * sinf(wheelStates[3].Frd_angle) + Vw_2_rad * sinf(wheelStates[1].Frd_angle) + Vw_3_rad * sinf(wheelStates[2].Frd_angle));
    state.Vw_extern = (-params->r/(4.0*params->R)) * (-Vw_1_rad * sinf(wheelStates[0].Frd_angle) + Vw_2_rad * cosf(wheelStates[1].Frd_angle) + Vw_3_rad * sinf(wheelStates[2].Frd_angle) - Vw_4_rad * cosf(wheelStates[3].Frd_angle));

    //state.Vx_extern =1.0;
    //state.Vy_extern =1.0;
    //state.Vw_extern =1.0;

    float CH2=DBUS.Remote.CH2/660.0*5.0;
    float CH3=DBUS.Remote.CH3/660.0*5.0;
    float dial=DBUS.Remote.Dial/660.0*5.0;
    if(fabs(CH2) < 0.1f) CH2 = 0;
    if(fabs(CH3) < 0.1f) CH3 = 0;
    if(fabs(dial) < 0.1f) dial = 0;

    state.Vx_dot_need=PID_Calculate(&state.Chassis_pid_x,state.Vx_extern,CH2);
    state.Vy_dot_need=PID_Calculate(&state.Chassis_pid_y,state.Vy_extern,CH3);
    state.Vw_dot_need=PID_Calculate(&state.Chassis_pid_w,state.Vw_extern,dial);

    //state.Vx_dot_need=1.5;
    //state.Vy_dot_need=1.5;
    //state.Vw_dot_need=1.5;



    x_dot_0=state.Vx_extern;
    y_dot_0=state.Vy_extern;
    w_dot_0=state.Vw_extern;

    x_ddot_0=state.Vx_dot_need;
    y_ddot_0=state.Vy_dot_need;
    w_ddot_0=state.Vw_dot_need;




}


void cal_wheel_torque(ChassisParams *params, SwerveWheelState wheelStates[4], ChassisState state)
{

    float R=params->R;
    float r=params->r;
    float m=params->m;
    float J=params->J;

    for (int i=0; i<4; i++)
    {
        float Pos_angle=wheelStates[i].Pos_angle;
        float Frd_angle=wheelStates[i].Frd_angle;



        //计算前馈力矩
        float ff_x=m*x_ddot_0*cosf(Frd_angle)*R;
        float ff_y=m*y_ddot_0*sinf(Frd_angle)*R;
        float ff_w=J*w_ddot_0*sinf(Frd_angle-Pos_angle)*R;

        float tau_forward=(ff_x+ff_y+ff_w)*r/(4*R);
        //tau_forward=0;

        //计算pid控制器
        float P_x=x_dot_0*cosf(Frd_angle);
        float P_y=y_dot_0*sinf(Frd_angle);
        float P_w=R*w_dot_0*sinf(Frd_angle-Pos_angle);
        float target=0;
              target=(P_x+P_y+P_w)/r;

       float tau_pid=0;
        tau_pid=PID_Calculate(&wheelStates[i].pid, wheelStates[i].speed, target);

        wheelStates[i].tau_need=(tau_forward+tau_pid)*600;
        if(wheelStates[i].tau_need > 16000) wheelStates[i].tau_need = 16000;
        if(wheelStates[i].tau_need < -16000) wheelStates[i].tau_need = -16000;
        wheelStates[i].tau_need=0;

    }


}

void cal_Swerve_torque(MOTOR_Typdef *Motor,SwerveWheelState wheelStates[4], ChassisParams *params,ChassisState state,SwerveState Swerve[4])
{

    float tau_need=0;

    for (int i=0; i<4; i++)
    {



        Vx_i=x_dot_0-params->R*w_dot_0*sinf(wheelStates[i].Pos_angle);
        Vy_i=y_dot_0+params->R*w_dot_0*cosf(wheelStates[i].Pos_angle);
        V_i=sqrtf(Vx_i*Vx_i+Vy_i*Vy_i);//速度向量的大小

        A_xi=x_ddot_0-params->R*(w_ddot_0*sinf(wheelStates[i].Pos_angle)+w_dot_0*w_dot_0*cosf(wheelStates[i].Pos_angle));
        A_yi=y_ddot_0+params->R*(w_ddot_0*cosf(wheelStates[i].Pos_angle)-w_dot_0*w_dot_0*sinf(wheelStates[i].Pos_angle));
        A_i=sqrtf(A_xi*A_xi+A_yi*A_yi);//加速度向量的大小

        target_angle=Swerve[i].last_angle_target;//默认保持上一次的目标角度

        //只有当加速度到达一定阈值（摇杆推的角度够大），才开始计算舵电机朝向与输出
        if (A_i > 0.05f)
        {
            target_angle = atan2f(A_yi, A_xi);
            Swerve[i].last_angle_target = target_angle; // 更新记忆
        }
        else if (V_i > 0.05f)
        {
            target_angle = atan2f(Vy_i, Vx_i);
            Swerve[i].last_angle_target = target_angle; // 更新记忆
        }


        //文档舵轮力矩计算公式中的最后一个，速度和加速度都为0时输出0力矩
        if (A_i==0.00)
        {
            tau_need=0;

        }

        //加速度不为0时，计算舵轮力矩输出
        else if (A_i>0.0001)
        {



            float err=target_angle-wheelStates[i].Frd_angle;
            //float err=0;

            //保证角度始终在正负PI之间
            while (err>PI)
            {
                err-=2*PI;
            }
            while (err<-PI)
            {
                err+=2*PI;
            }
/******************************************************************************/
            //防止电机出现90度以上的大转角
                    if (err > PI/2.0f)
                    {
                        err -= PI;
                        wheelStates[i].forward_3508 = -1;
                    }
                    else if (err < -PI/2.0f)
                    {
                        err += PI;
                        wheelStates[i].forward_3508 = -1;
                    }
            /******************************************************************************/

            if(err > -0.002f && err < 0.002f) {
                err = 0;
            }

            target_speed=0.0f+PID_Calculate(&Swerve[i].Swerve_pid_angle, 0, err);
                //target_speed=1.0;
            tau_need=PID_Calculate(&Swerve[i].Swerve_pid_speed,Swerve[i].speed , target_speed);


        }



        //除上述情况以外的情况都用这个，这个不是单纯的pid环，加入了前馈
        else
      {


          float err=target_angle-wheelStates[i].Frd_angle;

            //控制角度在正负PI间
          while (err>PI)
          {
              err-=2*PI;
          }
          while (err<-PI)
          {
              err+=2*PI;
          }


            //此处与上文同理
            if (err > PI/2.0f)
            {
                err -= PI;          // 角度只需要转到反方向
                wheelStates[i].forward_3508 = -1;
            }
            else if (err < -PI/2.0f)
            {
                err += PI;
                wheelStates[i].forward_3508 = -1;
            }




            if(err > -0.02f && err < 0.02f) {
                err = 0;
            }

          float part1 = state.Vx_extern * state.Vy_dot_need
            - state.Vy_extern * state.Vx_dot_need
            - state.Vw_extern * (state.Vx_extern * state.Vx_extern + state.Vy_extern * state.Vy_extern);

          float part2 = params->R * cosf(wheelStates[i].Pos_angle)
            * (state.Vw_dot_need * state.Vx_extern
            - state.Vw_extern * (state.Vx_dot_need + state.Vw_extern * state.Vy_extern));

          float part3 = params-> R * sinf(wheelStates[i].Pos_angle)
                       * (state.Vw_dot_need * state.Vy_extern
                       - state.Vw_extern * (state.Vy_dot_need + state.Vw_extern * state.Vx_extern));

          float w_dot_ff=(part1+part2+part3)/(V_i*V_i);

          target_speed=w_dot_ff+PID_Calculate(&Swerve[i].Swerve_pid_angle, 0, err);

          tau_need=PID_Calculate(&Swerve[i].Swerve_pid_speed,Swerve[i].speed , target_speed);



      }
            Swerve[i].tau_need=tau_need*13000;
    }

}

void stop(DBUS_Typedef *DBUS,SwerveState Swerve[4],SwerveWheelState wheel[4])
{

        for (int i=0;i<4;i++)
        {
            Swerve[i].tau_need=0;
            wheel[i].tau_need=0;

        }

}

