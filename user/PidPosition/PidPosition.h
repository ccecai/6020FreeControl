#ifndef __PIDPOSITION_H
#define __PIDPOSITION_H
#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"
#include "stdio.h"
#include "usart.h"
#include "CanMotor.h"
#include "PidControl.h"
typedef struct{
    float position_kp;
    float position_ki;
    float position_kd;
    float bias;
    float integral_bias;
    float last_bias;
    float TargetPosition;
    float CurrentPosition;
    float Velocity;
    int TargetVelocity;
    int circle;
    int PwmOut;
}PidPosition;

float PidPosition_PwmGet(void);
void PidPosition_Init(void);
int Velocity_Restrict(int PWM_P, int TargetVelocity);
extern uint8_t TX_BUF[100];
extern int32_t encoder;
extern char tx_buf[100];
extern moto_info_t motor_yaw_info;
extern pid_struct_t gimbal_yaw_speed_pid;
extern pid_struct_t gimbal_yaw_angle_pid;
extern int angle,last_angle,total_cnt,first_angle,total_angle;

#ifdef __cplusplus
}
#endif
#endif
