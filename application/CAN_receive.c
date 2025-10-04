/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "detect_task.h"

#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
float _207_angle[2];

#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }
static motor_measure_t motor_chassis[4];  //���̵��
static motor_measure_t motor_gimbal[2];   //��̨���
static motor_measure_t motor_trigger;     //�������
static motor_measure_t motor_friction[4]; //Ħ���ֵ��
static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
static CAN_TxHeaderTypeDef cap_tx_message;
static uint8_t cap_can_send_data[8];


float PowerData[7];
	
int16_t round_207;
float this_angle_207;

/**
 * @brief           ����3508����ۼ���ת�Ƕ�
 * @param[out]      motor:����ṹ����ָ��
 */
void Calc_motor_Angle(motor_measure_t *motor)
{
  if (motor->ecd - motor->last_ecd > 4095.5)
  {
    motor->round--;
  }
  else if (motor->ecd - motor->last_ecd < -4095.5)
  {
    motor->round++;
  }
  motor->angle = (motor->round * ANGLE_T + motor->last_ecd) * 0.04395067757f/19; //ת��Ϊ360��һȦ
}

/**
 * @brief          hal��CAN�ص�����,���յ������
 * @param[in]      hcan:CAN���ָ��
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;

  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  if (hcan == &hcan2)
  {
    switch (rx_header.StdId)
    {
    case CAN_3508_M1_ID:
    {
      get_motor_measure(&motor_chassis[0], rx_data); //��ȡ����
      // Calc_motor_Angle(&motor_chassis[0]);//�Ƕȼ���
      detect_hook(CHASSIS_MOTOR1_TOE); //������
      break;
    }
    case CAN_3508_M2_ID:
    {
      get_motor_measure(&motor_chassis[1], rx_data);
      // Calc_motor_Angle(&motor_chassis[1]);
      detect_hook(CHASSIS_MOTOR2_TOE);
      break;
    }
    case CAN_3508_M3_ID:
    {
      get_motor_measure(&motor_chassis[2], rx_data);
      // Calc_motor_Angle(&motor_chassis[2]);
      detect_hook(CHASSIS_MOTOR3_TOE);
      break;
    }
    case CAN_3508_M4_ID:
    {
      get_motor_measure(&motor_chassis[3], rx_data);
      // Calc_motor_Angle(&motor_chassis[3]);      detect_hook(CHASSIS_MOTOR4_TOE);
      break;
    }


		case CAN_SUPERCAP_ID:
    {

    extern float PowerData[7];  
    uint16_t *pPowerData = (uint16_t *)rx_data;
    PowerData[0] = pPowerData[0];                        // pPowerData[0]
    PowerData[1] = (rx_data[0]<<8|rx_data[1])/1000.0f ;  // capacitor_voltage
			
    PowerData[2] = (int16_t)(rx_data[2]<<8|rx_data[3])/1000.0f; //capacitor_current
    PowerData[3] = rx_data[4];                           // battery_power
    PowerData[4] = rx_data[5];                           // limit_power
    PowerData[5] = rx_data[6];                           // capacitor_level

    break;
      break;
    }

    default:
    {
      break;
    }
    }
  }
  else if (hcan == &hcan1)
  {
    switch (rx_header.StdId)
    {
    case CAN_FRONT_LEFT_FRICTION_ID:
    {
      get_motor_measure(&motor_friction[0], rx_data); //��ȡ����
      detect_hook(FRONT_LEFT_FRICTION_MOTOR_TOE);           //������
      break;
    }
    case CAN_FRONT_RIGHT_FRICTION_ID:
    {
      get_motor_measure(&motor_friction[1], rx_data); //��ȡ����
      detect_hook(FRONT_RIGHT_FRICTION_MOTOR_TOE);          //������
      break;
    }
	 case CAN_PIT_MOTOR_ID:
    {
      get_motor_measure(&motor_gimbal[1], rx_data);
      // Calc_motor_Angle(&motor_gimbal[1]);
      detect_hook(PITCH_GIMBAL_MOTOR_TOE);
      break;
    }
		case CAN_TRIGGER_MOTOR_ID:
    {
      get_motor_measure(&motor_trigger, rx_data);
      Calc_motor_Angle(&motor_trigger);
      detect_hook(TRIGGER_MOTOR_TOE);
      _207_angle[1] = _207_angle[0];
      _207_angle[0] = motor_trigger.ecd * 0.044;
      if (_207_angle[0] - _207_angle[1] > 180)
      {
        round_207--;
      }
      else if (_207_angle[0] - _207_angle[1] < -180)
      {
        round_207++;
      }
      this_angle_207 = round_207 * 360 + _207_angle[0];
      break;
    }
	case CAN_YAW_MOTOR_ID:
    {
      get_motor_measure(&motor_gimbal[0], rx_data);
      // Calc_motor_Angle(&motor_gimbal[0]);
      detect_hook(YAW_GIMBAL_MOTOR_TOE);
      break;
    }


    default:
    {
      break;
    }
    }
  }
}

/**
 * @brief Ħ���ֿ��������ͺ��� can1
 *
 * @param left_friction
 * @param rigit_friction
 * @param rev
 */
void CAN_cmd_friction(int16_t left_friction, int16_t rigit_friction)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_FRICTION_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = 0;
  gimbal_can_send_data[1] = 0;
  gimbal_can_send_data[2] = (left_friction >> 8);
  gimbal_can_send_data[3] = left_friction;
  gimbal_can_send_data[4] = (rigit_friction >> 8);
  gimbal_can_send_data[5] = rigit_friction;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  //HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
	HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}


/**
 * @brief ����������������ͺ���   can2    ID 
 *
 * @param shoot
 */
void CAN_cmd_shoot(int16_t shoot)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = 0x1FF;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (shoot >> 8);//0;
  gimbal_can_send_data[1] = (shoot >> 8);//0;
  gimbal_can_send_data[2] = 0;
  gimbal_can_send_data[3] = 0;
  gimbal_can_send_data[4] = //(shoot >> 8);
  gimbal_can_send_data[5] = //shoot;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  //HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
	HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208) can1
 * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
 * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
 * @retval         none
 */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch)
{
	// Pitch  Can1    ���ID 4 0x208
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = 0x2FF;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
//  gimbal_can_send_data[0] = (yaw >> 8);
//  gimbal_can_send_data[1] = yaw;
	gimbal_can_send_data[0] = 0;
  gimbal_can_send_data[1] = 0;
  gimbal_can_send_data[2] = (pitch >> 8);
  gimbal_can_send_data[3] = pitch;
  gimbal_can_send_data[4] = (yaw >> 8);
  gimbal_can_send_data[5] = yaw;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);

}

/**
 * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204) can1
 * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
 * @retval         none
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief     ���ͳ��������趨����
 * @param Power   30W~250W
 * 10Hz
 */
void super_cap_send_power(uint16_t power)
{
	 uint32_t send_mail_box;
   cap_tx_message.StdId = CAN_SUPERCAP_ID;
   cap_tx_message.IDE = CAN_ID_STD;
   cap_tx_message.RTR = CAN_RTR_DATA;
   cap_tx_message.DLC = 0x08;
   cap_can_send_data[0]=(uint8_t)power; //��������
   cap_can_send_data[1]=(uint8_t)(power*100.0f)%100; //С������
   cap_can_send_data[2]=0;
   cap_can_send_data[3]=0;
   cap_can_send_data[4]=0;
   cap_can_send_data[5]=0;
   cap_can_send_data[6]=0;
   cap_can_send_data[7]=0;
   HAL_CAN_AddTxMessage(&hcan2,&cap_tx_message, cap_can_send_data, &send_mail_box);

}
/**
 * @brief          ����yaw 6020�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_gimbal[0];
}

/**
 * @brief          ����pitch 6020�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_gimbal[1];
}

/**
 * @brief          ���ز������ 2006�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_trigger;
}

const motor_measure_t *get_left_friction_motor_measure_point(void)
{
  return &motor_friction[0];
}

const motor_measure_t *get_right_friction_motor_measure_point(void)
{
  return &motor_friction[1];
}

/**
 * @brief          ���ص��̵�� 3508�������ָ��
 * @param[in]      i: ������,��Χ[0,3]
 * @retval         �������ָ��
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}

