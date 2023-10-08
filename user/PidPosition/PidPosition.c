#include "PidPosition.h"

float position_kp;
float position_ki;
float position_kd;
float CurrentPosition;
int circle;

float PidPosition_PwmGet(void)
{
    float bias,TargetPosition,Velocity;
    static float integral_bias,last_bias;
    TargetPosition=circle*8192;
    bias=TargetPosition-total_angle;
    integral_bias+=bias;

    if(integral_bias> 750) integral_bias= 750;
    if(integral_bias<-750) integral_bias=-750;

    gimbal_yaw_angle_pid.output = position_kp*bias+position_ki*integral_bias+position_kd*(bias-last_bias);

    if(gimbal_yaw_angle_pid.output > 20000)
    {
        gimbal_yaw_angle_pid.output = 20000;
    }

    sprintf(tx_buf,"%f\r\n",bias);
    HAL_UART_Transmit(&huart1,(uint8_t *)tx_buf,10,0xffff);

    last_bias=bias;
    //return gimbal_yaw_angle_pid.output;


}

int Velocity_Restrict(int PWM_P, int TargetVelocity)
{
    if     (PWM_P>+TargetVelocity*76) PWM_P=+TargetVelocity*76;
    else if(PWM_P<-TargetVelocity*76) PWM_P=-TargetVelocity*76;
    else PWM_P=PWM_P;
    return PWM_P;
}

void PidPosition_Init(void)
{
    position_kp = 130;
    position_ki = 0.01f;
    position_kd = 20;
    CurrentPosition = 0;
    circle = 10;
}