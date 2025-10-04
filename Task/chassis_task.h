/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2



//����PID��������
//����ٶ�PID
#define CHASSIS_MOTOR_SPEED_KP 9000//5000//9000//15000 10000
#define CHASSIS_MOTOR_SPEED_KI 0.0125
#define CHASSIS_MOTOR_SPEED_KD 0//0.1
extern uint16_t CHASSIS_MOTOR_SPEED_MAX_OUT; //��3200(1.2)//��2600��1.3��//��2100��1.1��//��2750(1.2)//��2250(1.0)//��2300��1.2��//��2500��1.2��//��3000(1.3)//ʮ4000(1.5)//һ2000//7000//14000  //15000
#define CHASSIS_MOTOR_SPEED_MAX_IOUT 1000//10000
//���̸�����̨��תPID
#define CHASSIS_ANGLE_KP 10.0f//12.5f //16.0f   //13.0f        //12.5f
#define CHASSIS_ANGLE_KI 0
#define CHASSIS_ANGLE_KD 0.005f//0.001f
#define CHASSIS_ANGLE_MAX_OUT 8.0f//8.5f            //8.0f
#define CHASSIS_ANGLE_MAX_IOUT 1.5f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f

// m3508ת���ɵ����ٶ�(m/s)�ı�����
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//�������̵������ٶ�
#define MAX_WHEEL_SPEED 3.7f//2.0f//3.0f//4.0f//8.0f 4.0f
//�����˶��������ǰ���ٶ�
extern float NORMAL_MAX_CHASSIS_SPEED_X; //2.0f//1.0f//2.0f//3.0//1.0f//2.0f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f //2.5  //1.5f

#define NORMAL_MAX_CHASSIS_SPEED_Z 6.0f//3.0f

//����С�����ٶ�
#define CHASSIS_TOP_SPEED 9//8  // 6.0f//7
//����С����ʱ���ƽ���ٶ�
#define TOP_MAX_CHASSIS_SPEED 4.0f

#define CAP_CHARGE_VOLTAGE_THRESHOLD 18.0f  // ���ݳ���ѹ��ֵ
#define CAP_LOW_VOLTAGE_THRESHOLD   18.0f   // ���ݵ�ѹ��ֵ
#define CAP_HIGH_CURRENT_THRESHOLD  8.0f    // ���ݸߵ�����ֵ
//#define INITIAL_POWER_LIMIT        40.0f    // ��ʼ�������� 40W


// ��ӹ��ʿ�����ر���
static float chassis_power_scale = 1.0f;    // ���̹�������ϵ��
//static float cap_ready = 0;              // ���ݾ�����־
static float discharge_current = 0.0f;       // �ŵ����
typedef enum
{
  CHASSIS_ZERO_FORCE = 0,   //��������, ��û�ϵ�����
  CHASSIS_NO_MOVE,          //���̱��ֲ���
  CHASSIS_FOLLOW_GIMBAL,    //���̸�����̨��ת
  CHASSIS_TOP,              
  CHASSIS_NO_FOLLOW_GIMBAL, //���̲�������̨��ת
  CHASSIS_OPEN              //ң������ֵ���Ա����ɵ���ֵ ֱ�ӷ��͵�can������
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  float accel;          //���ٶ�
  float speed;          //��ǰ�ٶ�
  float speed_set;      //�趨�ٶ�
  int16_t give_current; //���͵���
} chassis_motor_t;

typedef struct
{
  const gimbal_motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����.
  const gimbal_motor_t *chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
  const float *chassis_INS_angle;            //��ȡ�����ǽ������ŷ����ָ��
  chassis_mode_e chassis_mode;               //���̿���״̬��
  chassis_motor_t motor_chassis[4];          //���̵������
  pid_type_def motor_speed_pid[4];           //���̵���ٶ�pid
  pid_type_def chassis_angle_pid;            //���̸���Ƕ�pid

  first_order_filter_type_t chassis_cmd_slow_set_vx; //ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_vy; //ʹ��һ�׵�ͨ�˲������趨ֵ

  float vx;                         //�����ٶ� ǰ������ ǰΪ������λ m/s
  float vy;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  float wz;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  float vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  float vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  float wz_set;                     //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  float chassis_relative_angle;     //��������̨����ԽǶȣ���λ rad
  float chassis_relative_angle_set; //���������̨���ƽǶ�
  float chassis_yaw_set;

  float vx_max_speed;  //ǰ����������ٶ� ��λm/s
  float vx_min_speed;  //���˷�������ٶ� ��λm/s
  float vy_max_speed;  //��������ٶ� ��λm/s
  float vy_min_speed;  //�ҷ�������ٶ� ��λm/s
  float top_max_speed; //С����ƽ������������ٶ�
  float top_min_speed; //С����ƽ�Ʒ���������ٶ�
  float chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
  float chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�
  float chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�
} chassis_move_t;

/**
 * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: ��
 * @retval         none
 */
extern void chassis_task(void const *pvParameters);

#endif
