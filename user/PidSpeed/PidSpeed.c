#include "PidSpeed.h"

float speed_kp=360;
float speed_ki=1.5;
float speed_kd;

int PidSpeed_PwmGet(int Encoder,int Target)
{
    int bias;
    static int PwmOut,last_bias;
    bias = Target-Encoder;
    PwmOut += speed_kp*(bias-last_bias)+speed_ki*bias;
    last_bias = bias;

    return PwmOut;
}



void PidSpeed_Init(void)
{
    PidSpeed speed;
    speed.speed_kp = 20;
    speed.speed_ki = 0;
    speed.speed_kd = 0;
    speed.bias = 0;
    speed.last_bias = 0;
    speed.PwmOut = 0;
    speed.TargetSpeed = 0;
}