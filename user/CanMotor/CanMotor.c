#include "CanMotor.h"

moto_info_t motor_yaw_info;
uint16_t can_cnt;
uint32_t msg_cnt = 0;
int angle,last_angle,total_cnt,first_angle;
int total_angle = 0;

void can_filter_init(void)//筛选器配置
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             rx_data[8];
    if(hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data

        switch(rx_header.StdId)
        {
            case 0x205:
            {
                if(msg_cnt == 0)
                {
                    motor_yaw_info.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
                    motor_yaw_info.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
                    motor_yaw_info.torque_current = ((rx_data[4] << 8) | rx_data[5]);
                    motor_yaw_info.temp           =   rx_data[6];
                    first_angle = motor_yaw_info.rotor_angle;
                    angle = motor_yaw_info.rotor_angle;
                    msg_cnt = 1;
                }


                else
                {

                    last_angle = angle;
                    motor_yaw_info.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
                    motor_yaw_info.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
                    motor_yaw_info.torque_current = ((rx_data[4] << 8) | rx_data[5]);
                    motor_yaw_info.temp           =   rx_data[6];
                    angle = motor_yaw_info.rotor_angle;
                    if(angle - last_angle > 4095)
                    {
                        total_cnt--;
                    }
                    else if(angle - last_angle < -4095)
                    {
                        total_cnt++;
                    }

                    total_angle = total_cnt * 8191 + angle - first_angle;
                }

                break;
            }

        }
    }
}

void set_GM6020_motor_voltage(CAN_HandleTypeDef* hcan,int16_t v1) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};

    tx_header.StdId = 0x1ff;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    tx_data[0] = (v1 >> 8);
    tx_data[1] = v1;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t *) CAN_TX_MAILBOX0);

}
void CAN_Init(CAN_HandleTypeDef* hcan)
{
    assert_param(hcan != NULL);
    HAL_CAN_Start(hcan);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    //注意使能中断
    HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CANMOTOR_Init(void)
{
    CAN_Init(&hcan1);
    CAN_Init(&hcan2);
    can_filter_init();
}

double msp(double x, double in_min, double in_max, double out_min, double out_max)//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi）
{
    return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

double Angle_Change(int Angle)
{
    //return Angle*0.01745329252;
    return Angle*0.0175;
}