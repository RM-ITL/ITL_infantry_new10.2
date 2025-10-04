	/**
 * @file USART_receive.c
 * @author ���廪
 * @brief �����жϽ��պ�������������Ƭ���������豸�Ĵ���ͨ������
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
auto_shoot_t auto_shoot = {0.0f, 0.0f}; //��������
user_send_data_t user_send_data;

//����ԭʼ���ݣ�Ϊ10���ֽڣ�����18���ֽڳ��ȣ���ֹDMA����Խ��
uint8_t usart1_rx_buf[2][USART1_RX_BUF_NUM];

/**
 * @brief �û����ݽ��
 *
 * @param buf ���ڽ�������ָ��
 * @param auto_shoot �������ݽṹָ��
 */
void user_data_solve(volatile const uint8_t *buf, auto_shoot_t *auto_shoot);

void user_usart_init(void)
{
    usart1_init(usart1_rx_buf[0], usart1_rx_buf[1], USART1_RX_BUF_NUM);
}

void USART1_IRQHandler(void)
{
    if (huart1.Instance->SR & UART_FLAG_RXNE) //���յ�����
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
            //ʧЧDMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = USART1_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;
            //�����趨���ݳ���
            huart1.hdmarx->Instance->NDTR = USART1_RX_BUF_NUM;
            //�趨������1
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            //ʹ��DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);

            if (this_time_rx_len == USER_FRAME_LENGTH)
            {
                //��ȡ����
                user_data_solve(usart1_rx_buf[0], &auto_shoot);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //ʧЧDMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = USART1_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;
            //�����趨���ݳ���
            huart1.hdmarx->Instance->NDTR = USART1_RX_BUF_NUM;
            //�趨������0
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            //ʹ��DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);
            if (this_time_rx_len == USER_FRAME_LENGTH)
            {
                //��ȡ����
                user_data_solve(usart1_rx_buf[1], &auto_shoot);//1
            }
        }
    }
}

/**
 * @brief �û����ݽ��
 *
 * @param buf ���ڽ�������ָ��
 * @param auto_shoot �������ݽṹָ��
 */
void user_data_solve(volatile const uint8_t *buf, auto_shoot_t *auto_shoot)
{
    //У��
    if (buf[0] == 0xFF && buf[5] == 0xFE)
    {
        auto_shoot->pitch_add = (0.0001f * ((float)(buf[2] << 8 | buf[1]))) - USART_PI;
        auto_shoot->yaw_add = (0.0001f * ((float)(buf[4] << 8 | buf[3]))) - USART_PI;
        detect_hook(USER_USART_DATA_TOE);
    }
}





int a = 0;//ģʽ�л���־λ��Ϊ0��ʾ������������Ϊ1��ʾ������̨����,2�������
float chassis_power1 = 0.0f;   				//���̹���
float chassis_power_buffer1 = 0.0f;   //���̻�������
extern ext_power_heat_data_t power_heat_data_t;
extern void get_chassis_power_and_buffer(float *current,float *volt,float *power, float *buffer);
void user_data_pack_handle()
{
	if(a==0)
	{
						//���͵�ǰ�����ٶ� ��λm/s
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
        user_send_data.enemy_color = 0;//��ɫ
    }
    else
    {
        user_send_data.enemy_color = 1;//��ɫ
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
