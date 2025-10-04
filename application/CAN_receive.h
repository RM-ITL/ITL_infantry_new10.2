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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan2
#define GIMBAL_CAN hcan2

#define ANGLE_T 8191
extern float PowerData[7];
//��������
extern float capacitor_voltage;    // ���ݵ�ѹ
extern float capacitor_current;    // ���ݵ���
extern uint8_t battery_power;      // ��ع���
extern uint8_t limit_power;        // ���ƹ���
extern uint8_t capacitor_level;    // ���ݵ���
extern float power;                // ���͵Ĺ���ֵ

/* CAN send and receive ID */
typedef enum
{
  CAN_CHASSIS_ALL_ID = 0x200,
  CAN_3508_M1_ID = 0x201,
  CAN_3508_M2_ID = 0x202,
  CAN_3508_M3_ID = 0x203,
  CAN_3508_M4_ID = 0x204,

  CAN_GIMBAL_ALL_ID = 0x2FF, //6020
     
  CAN_TRIGGER_MOTOR_ID = 0x205,
	
	CAN_GIMBAL_2_ID=0x1FF,
  CAN_YAW_MOTOR_ID = 0x20B,//CAN_YAW_MOTOR_ID = 0x209,
	CAN_PIT_MOTOR_ID = 0x20A,
	CAN_SUPERCAP_ID = 0x0FF,    
 
	
  CAN_FRICTION_ALL_ID = 0x200,
  CAN_FRONT_LEFT_FRICTION_ID = 0x202,
  CAN_FRONT_RIGHT_FRICTION_ID = 0x203,
  //CAN_LAST_LEFT_FRICTION_ID = 0x203,
  //CAN_LAST_RIGHT_FRICTION_ID = 0x204,	
} can_msg_id_e;

// rm motor data
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
  int round;
  float angle;
} motor_measure_t;

/**
 * @brief Ħ���ֿ��������ͺ��� can2
 *
 * @param left_friction
 * @param rigit_friction
 */
extern void CAN_cmd_friction(int16_t left_friction, int16_t rigit_friction);

/**
 * @brief ����������������ͺ��� can1
 *
 * @param shoot
 */
extern void CAN_cmd_shoot(int16_t shoot);

/**
 * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
 * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
 * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
 * @retval         none
 */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch);

/**
 * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
 * @retval         none
 */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
 * @brief          ����yaw 6020�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
 * @brief          ����pitch 6020�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

extern const motor_measure_t *get_left_friction_motor_measure_point(void);
extern const motor_measure_t *get_right_friction_motor_measure_point(void);

/**
 * @brief          ���ز������ 2006�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
 * @brief          ���ص��̵�� 3508�������ָ��
 * @param[in]      i: ������,��Χ[0,3]
 * @retval         �������ָ��
 */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
 * @brief           �򳬼����ݷ��͹���
 * @param[in]      Power   0~3000 ��Ӧ0W~30W
 * @retval         �������ָ��
 */
extern void super_cap_send_power(uint16_t Power);





#endif
