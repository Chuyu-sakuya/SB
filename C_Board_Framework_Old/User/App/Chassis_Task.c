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
double GoalAngle[4];
double ExternAngle[4];
double LastAngle[4];
double DeltaAngle[4];
uint8_t Length;
void swerveWheelResolve(double *Wheel_Rpm,float Vx_truth,float Vy_truth,float Vw_truth)
{


    //检测轮速，轮速低于极小值时，舵电机保持上次状态
    if (Wheel_Rpm[0]<1e-6&&Wheel_Rpm[1]<-1e-6&&Wheel_Rpm[2]<-1e-6&&Wheel_Rpm[3]<-1e-6)
    {
        for (int i=0;i<4;i++)
        {
            ExternAngle[i]=LastAngle[i];
        }
    }

    //简单的速度计算
    double V1=(Vx_truth-Vw_truth*Length);
    double V2=(Vy_truth+Vw_truth*Length);
    double V3=(Vx_truth+Vw_truth*Length);
    double V4=(Vy_truth-Vw_truth*Length);
    //简单的角度计算
    GoalAngle[0]=atan2(V2,V3)*180/PI;
    GoalAngle[1]=atan2(V2,V1)*180/PI;
    GoalAngle[2]=atan2(V4,V3)*180/PI;
    GoalAngle[3]=atan2(V4,V1)*180/PI;

    for (int i=0;i<4;i++)
    {

    }

    //计算最小旋转角度，舵电机不会旋转90度以上的角度
   for (uint8_t i=0;i<4;i++)
   {
       DeltaAngle[i]=GoalAngle[i]-ExternAngle[i];



       if (DeltaAngle[i]>90||DeltaAngle[i]<-90)
       {

           if (DeltaAngle[i]>90){ GoalAngle[i]=-(180-GoalAngle[i]);}
           if (DeltaAngle[i]<-90){ GoalAngle[i]=GoalAngle[i]+180;}

       }


   }


    //更新上次角度
    for (uint8_t i=0;i<4;i++)
    {
        LastAngle[i]=ExternAngle[i];
    }


    //将速度分到轮子上
    Wheel_Rpm[0]=sqrt(V3*V3+V2*V2);
    Wheel_Rpm[1]=sqrt(V1*V1+V2*V2);
    Wheel_Rpm[2]=sqrt(V3*V3+V4*V4);
    Wheel_Rpm[3]=sqrt(V1*V1+V4*V4);



}