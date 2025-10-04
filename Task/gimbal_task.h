/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

/* ******************************************************************
                        AGV_CHASIS
****************************************************************** */
//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 201
//��̨������Ƽ�� 1ms
#define GIMBAL_CONTROL_TIME 1

// yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL LEFT_SWITCH

//��ͷ��̨�ٶ�
#define TURN_SPEED 0.04f
//���԰�����δʹ��
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_DEADBAND 10

//������
#define YAW_RC_SEN   -0.000005f
#define PITCH_RC_SEN  0.000006f // 0.005

#define YAW_MOUSE_SEN 0.00009f  //0.00009f  //0.00005f
#define PITCH_MOUSE_SEN 0.000055f  //0.00003f//0.00010f

#define YAW_ENCODE_SEN 0.01f
#define PITCH_ENCODE_SEN 0.01f

//�ж�ң�����������ʱ���Լ�ң�����������жϣ�������̨yaw����ֵ�Է�������Ư��
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

///////////////////////////����������ǰ�װ����////////////////////////////////
#define PITCH_TURN 0
#define PITCH_OUT_TURN 0
#define YAW_TURN 1
#define YAW_OUT_TURN 1


/////////////////////////////��̨pid����///////////////////////////////////
// pitch��ǶȻ������ԽǶȣ�
#define PITCH_ABSOLUTE_ANGLE_KP 1.8f//2.8f//3.5f//5.0f//30.0f//10.0f//7.7f//7.7f//10.7f     //6.7f//56.0f
#define PITCH_ABSOLUTE_ANGLE_KI 0.00001f
#define PITCH_ABSOLUTE_ANGLE_KD 0.67f//0.0001f
#define PITCH_ABSOLUTE_ANGLE_MAX_OUT 20.0f    //20.0f
#define PITCH_ABSOLUTE_ANGLE_MAX_IOUT  1000.0f //�������ģ�debug
// pitch���ٶȻ�
#define PITCH_GYRO_KP 5500.0f//4500.0f//5500.0f//12000.0f//12000.0f              //-5500.0f    //-6800
#define PITCH_GYRO_KI 0.0f 
#define PITCH_GYRO_KD 10.2f              //10.8f
#define PITCH_GYRO_MAX_OUT 20000.0f
#define PITCH_GYRO_MAX_IOUT 0  //1000.0f

// yaw��ǶȻ������ԽǶȣ�
#define YAW_ABSOLUTE_ANGLE_KP   -10.0f//10.12f //7.98f  //5.0f //10.18f//14.0f//4.2f   //5.6f     //  8.39f     //8.39f    //5.6f  10.11f
#define YAW_ABSOLUTE_ANGLE_KI   0.0f   //0.0001f
#define YAW_ABSOLUTE_ANGLE_KD   0.6f//0.5f//-0.95f  //-1.0f   //-0.95f   //-1.18f  //0.01f  0.2
#define YAW_ABSOLUTE_ANGLE_MAX_OUT 5.0f  //5.0f   //8.0f //15.0f   //9.0f   5.0f
#define YAW_ABSOLUTE_ANGLE_MAX_IOUT 1.0f  //3.0f
// yaw���ٶȻ�
#define YAW_GYRO_KP 20000.0f//21000.0f     
#define YAW_GYRO_KI 0.0f  //0.0f
#define YAW_GYRO_KD 0.5f
#define YAW_GYRO_MAX_OUT 16000.0f
#define YAW_GYRO_MAX_IOUT 1000.0f

/////////////////////////��̨��λ����ֵ�����趨////////////////////////////
#define GIMBAL_YAW_OFFSET_ECD          0x101F//6851//4741//6666//6666//5.112758��������6630//6666//0x1AFF-0x0400 //0.7762f    0x095c(-30)  0x19CF    0x1AFF-0x0400=19FF(-5)
#define GIMBAL_YAW_MAX_ECD             5.0f  //���
#define GIMBAL_YAW_MIN_ECD             -5.0f //���
#define GIMBAL_PITCH_OFFSET_ECD        0x0C6E//2.419f//0x0C52//-0.09f//0x4500//0x1806   //6150
#define GIMBAL_PITCH_MAX_ECD           -0.897377551f//1.492f//0x0956//5.360f//0.143f//-0.45f// -0.403f    //0.38f   //0.38f
#define GIMBAL_PITCH_MIN_ECD            0.45562622f//2.770f//0x0E29//-5.40f//-0.75f//1.16f  // -1.40f// -0.666f   //-0.68f//-0.95f								

//�������ֵת���ɽǶ�ֵ
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

typedef enum
{
  GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����      ��̨��������ģʽ
  GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
  GIMBAL_MOTOR_ENCONDE  //�������ֵ�Ƕȿ���
} gimbal_motor_mode_e;

typedef struct
{
  float kp;
  float ki;
  float kd;

  float set;
  float get;
  float err;

  float max_out;
  float max_iout;

  float Pout;
  float Iout;
  float Dout;

  float out;
} gimbal_PID_t;

typedef struct
{
  const motor_measure_t *gimbal_motor_measure;
  gimbal_PID_t gimbal_motor_absolute_angle_pid;
  gimbal_PID_t gimbal_motor_relative_angle_pid;
  pid_type_def gimbal_motor_gyro_pid;
  gimbal_motor_mode_e gimbal_motor_mode;
  gimbal_motor_mode_e last_gimbal_motor_mode;
  float offset_ecd;
  float max_relative_angle; // rad
  float min_relative_angle; // rad

  float relative_angle;     // rad
  float relative_angle_set; // rad     //�����Ƕ�
  float absolute_angle;     // rad
  float absolute_angle_set; // rad
  float motor_gyro;         // rad/s
  float motor_gyro_set;
  float motor_speed;
  float raw_cmd_current;
  float current_set;
  int16_t given_current;
} gimbal_motor_t;

typedef struct
{
  gimbal_motor_t gimbal_yaw_motor;
  gimbal_motor_t gimbal_pitch_motor;
} gimbal_control_t;

extern gimbal_control_t gimbal_control;

/**
 * @brief          ����yaw �������ָ��
 * @param[in]      none
 * @retval         yaw���ָ��
 */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
 * @brief          ����pitch �������ָ��
 * @param[in]      none
 * @retval         pitch
 */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
 * @brief ��̨��̬���ݷ��͵���λ��
 *
 */
extern  void gimbal_data_send(void);

extern void gimbal_task(void const *pvParameters);

/**
 * @brief ��̨��̬���ݷ��͵���λ��
 *
 */
extern void gimbal_data_send(void);

#endif
