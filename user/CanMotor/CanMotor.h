#ifndef __CANMOTOR_H
#define __CANMOTOR_H

#include "main.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"
#include "stdio.h"
#include "usart.h"

typedef struct
{
    uint16_t can_id;//电机ID
    int16_t  set_voltage;//设定的电压值
    uint16_t rotor_angle;//机械角度
    int16_t  rotor_speed;//转速
    int16_t  torque_current;//扭矩电流
    uint8_t  temp;//温度
}moto_info_t;

void can_filter_init(void);
void set_GM6020_motor_voltage(CAN_HandleTypeDef* hcan,int16_t v1);
void CAN_Init(CAN_HandleTypeDef* hcan);
void CANMOTOR_Init(void);
double msp(double x, double in_min, double in_max, double out_min, double out_max);
double Angle_Change(int Angle);
int32_t Encoder_Get(void);
extern int32_t encoder;
extern char tx_buf[100];
#endif

