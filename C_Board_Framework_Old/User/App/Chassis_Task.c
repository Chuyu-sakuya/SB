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
double HeadingAngle[4];
uint8_t Length;
void swerveWheelResolve(double *Wheel_Rpm,float Vx_truth,float Vy_truth,float Vw_truth)
{


    double V1=Vx_truth-Vw_truth*Length;
    double V2=Vy_truth+Vw_truth*Length;
    double V3=Vx_truth+Vw_truth*Length;
    double V4=Vy_truth-Vw_truth*Length;

    HeadingAngle[0]=atan2(V2,V3)*180/PI;
    HeadingAngle[1]=atan2(V2,V1)*180/PI;
    HeadingAngle[2]=atan2(V4,V3)*180/PI;
    HeadingAngle[3]=atan2(V4,V1)*180/PI;

    Wheel_Rpm[0]=sqrt(V3*V3+V2*V2);
    Wheel_Rpm[1]=sqrt(V1*V1+V2*V2);
    Wheel_Rpm[2]=sqrt(V3*V3+V4*V4);
    Wheel_Rpm[3]=sqrt(V1*V1+V4*V4);



}