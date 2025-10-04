	/**
 * @file USART_receive.c
 * @author 何清华
 * @brief 串口中断接收函数，用来处理单片机与其他设备的串口通信数据
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "USART_receive.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"
#include "referee.h"
#include "usart.h"
#include "chassis_task.h"
#include "chassis_power_control.h"
extern chassis_move_t chassis_move;
extern UART_HandleTypeDef huart1;
extern float current;
auto_shoot_t auto_shoot = {0.0f, 0.0f}; //自瞄数据
user_send_data_t user_send_data;

//接收原始数据，为10个字节，给了18个字节长度，防止DMA传输越界
uint8_t usart1_rx_buf[2][USART1_RX_BUF_NUM];

/**
 * @brief 用户数据解包
 *
 * @param buf 串口接收数据指针
 * @param auto_shoot 自瞄数据结构指针
 */
void user_data_solve(volatile const uint8_t *buf, auto_shoot_t *auto_shoot);

void user_usart_init(void)
{
    usart1_init(usart1_rx_buf[0], usart1_rx_buf[1], USART1_RX_BUF_NUM);
}

void USART1_IRQHandler(void)
{
    if (huart1.Instance->SR & UART_FLAG_RXNE) //接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if (USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
            //失效DMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = USART1_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;
            //重新设定数据长度
            huart1.hdmarx->Instance->NDTR = USART1_RX_BUF_NUM;
            //设定缓冲区1
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            //使能DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);

            if (this_time_rx_len == USER_FRAME_LENGTH)
            {
                //读取数据
                user_data_solve(usart1_rx_buf[0], &auto_shoot);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //失效DMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = USART1_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;
            //重新设定数据长度
            huart1.hdmarx->Instance->NDTR = USART1_RX_BUF_NUM;
            //设定缓冲区0
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            //使能DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);
            if (this_time_rx_len == USER_FRAME_LENGTH)
            {
                //读取数据
                user_data_solve(usart1_rx_buf[1], &auto_shoot);//1
            }
        }
    }
}

/**
 * @brief 用户数据解包
 *
 * @param buf 串口接收数据指针
 * @param auto_shoot 自瞄数据结构指针
 */
void user_data_solve(volatile const uint8_t *buf, auto_shoot_t *auto_shoot)
{
    //校验
    if (buf[0] == 0xFF && buf[5] == 0xFE)
    {
        auto_shoot->pitch_add = (0.0001f * ((float)(buf[2] << 8 | buf[1]))) - USART_PI;
        auto_shoot->yaw_add = (0.0001f * ((float)(buf[4] << 8 | buf[3]))) - USART_PI;
        detect_hook(USER_USART_DATA_TOE);
    }
}





int a = 0;//模式切换标志位，为0表示传输电机参数，为1表示传输云台数据,2输出功率
float chassis_power1 = 0.0f;   				//底盘功率
float chassis_power_buffer1 = 0.0f;   //底盘缓冲能量
extern ext_power_heat_data_t power_heat_data_t;
extern void get_chassis_power_and_buffer(float *current,float *volt,float *power, float *buffer);
void user_data_pack_handle()
{
	if(a==0)
	{
						//发送当前底盘速度 单位m/s
		ANO_DT_send_int16((int16_t)(chassis_move.motor_chassis[0].speed*100), (int16_t)(chassis_move.motor_chassis[1].speed*100),
		(int16_t)(chassis_move.motor_chassis[2].speed*100), (int16_t)(chassis_move.motor_chassis[3].speed*100), 
		power_heat_data_t.chassis_current, power_heat_data_t.chassis_power,power_heat_data_t.chassis_power_buffer, 0 );
//		get_chassis_power_and_buffer(&chassis_power1, &chassis_power_buffer1);
//		ANO_DT_send_int16((int16_t)chassis_power1, (int16_t)chassis_power_buffer1	,0, 0, 0, 0, 0, 0 );
	}
	else
	{
		static uint8_t tx_buf[8];
    if (get_robot_id() <= 7)
    {
        user_send_data.enemy_color = 0;//红色
    }
    else
    {
        user_send_data.enemy_color = 1;//蓝色
    }

    tx_buf[0] = 0xFF;

    tx_buf[1] = ((uint16_t)(10000 * (*user_send_data.gimbal_pitch_angle + USART_PI)) >> 8);
    tx_buf[2] = (uint16_t)(10000 * (*user_send_data.gimbal_pitch_angle + USART_PI));
    tx_buf[3] = ((uint16_t)(10000 * (*user_send_data.gimbal_yaw_gyro + USART_PI)) >> 8);
    tx_buf[4] = (uint16_t)(10000 * (*user_send_data.gimbal_yaw_gyro + USART_PI));

    tx_buf[5] = user_send_data.enemy_color;

    tx_buf[6] = 0xFE;
		ANO_DT_send_int16(tx_buf[0], tx_buf[1],
		tx_buf[2], tx_buf[3], tx_buf[4], tx_buf[5], tx_buf[6], 0 );		
    //usart1_tx_dma_enable(tx_buf, 7);		
//	if(a==2)
//	{
//		//get_chassis_power_and_buffer(&chassis_power1, &chassis_power_buffer1);
//		ANO_DT_send_int16((int16_t)chassis_power1, (int16_t)chassis_power_buffer1	,0, 0, 0, 0, 0, 0 );
//	}
  	}
		


		//vTaskDelay(2000);
}
