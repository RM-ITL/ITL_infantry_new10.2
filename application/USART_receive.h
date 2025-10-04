/**
 * @file USART_receive.h
 * @author 何清华
 * @brief 串口中断接收函数，用来处理单片机与其他设备的串口通信数据
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef USART_RECEIVE_H
#define USART_RECEIVE_H

#include "struct_typedef.h"

#define USART1_RX_BUF_NUM 18u
#define USER_FRAME_LENGTH 6u

#define USART_PI 3.1416f

typedef struct
{
    float pitch_add;
    float yaw_add;
} auto_shoot_t;

typedef struct
{
    float *gimbal_pitch_angle;
    float *gimbal_yaw_gyro;
    uint8_t enemy_color;
} user_send_data_t;

extern auto_shoot_t auto_shoot;
extern user_send_data_t user_send_data;
void user_usart_init(void);
extern void user_data_pack_handle(void);

#endif
