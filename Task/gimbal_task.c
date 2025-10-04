/**
 * @file gimbal_task.c/h
 * @author ���廪,������
 * @brief ��̨���������߳�
 * @version 0.1
 * @date 2022-03-06
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
 
 /*
***********************ĳ�䲻�ţ����϶���֮ͷ����֮���ţ���л����***********************
*
                        1 1 1 1 1 1 1 1 1 1                 1
                                          1                 1
                                          1                 1
                                          1                 1
                                          1                 1
                                          1                 1
                                          1                 1
                                          1                 1
                        1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 
                        1                 1
                        1                 1
                        1                 1
                        1                 1
                        1                 1
                        1                 1
                        1                 1
                        1                 1 1 1 1 1 1 1 1 1 1 
												
***********************ĳ�䲻�ţ����϶���֮ͷ����֮���ţ���л����***********************

*/
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "cmsis_os.h"
#include "main.h"
#include "INS_task.h"
#include "arm_math.h"
#include "detect_task.h"
#include "user_lib.h"
#include "USART_receive.h"
#include "config.h"
#include "chassis_task.h"
#include "usart.h"
#define debug_mode 1
extern chassis_move_t chassis_move;
// PID�������
#define gimbal_total_pid_clear(gimbal_clear)                                               \
  {                                                                                        \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
    PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                           \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
    PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
  }

/**
 * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
 * @retval         none
 */
static void gimbal_init(void);

/**
 * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
 * @retval         none
 */
static void gimbal_set_mode(void);

/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @retval         none
 */
static void gimbal_feedback_update(void);

/**
 * @brief          ����ecd��offset_ecd֮�����ԽǶ�
 * @param[in]      ecd: �����ǰ����
 * @param[in]      offset_ecd: �����ֵ����
 * @retval         ��ԽǶȣ���λrad
 */
static float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

/**
 * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
 * @retval         none
 */
static void gimbal_set_control(void);

/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @retval         none
 */
static void gimbal_control_loop(void);

/**
 * @brief ��̨��̬���ݷ��͵���λ��
 *
 */
void gimbal_data_send(void);

/**
 * @brief          ��GIMBAL_MOTOR_GYROģʽ�����ƽǶ��趨,��ֹ�������
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float add);

/**
 * @brief          ��GIMBAL_MOTOR_ENCONDEģʽ�����ƽǶ��趨,��ֹ�������
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float add);

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief ��̨PID��ʼ������
 *
 * @param pid PID���ݽṹָ��
 * @param kp
 * @param ki
 * @param kd
 * @param maxout ������
 * @param max_iout ���������
 */
static void gimbal_PID_init(gimbal_PID_t *pid, float kp, float ki, float kd, float maxout, float max_iout);

/**
 * @brief ��̨PID���㺯��
 *
 * @param pid
 * @param get
 * @param set
 * @param error_delta ���΢���ֱ�ӴӴ�������ȡ��������
 * @return fp32
 */
static float gimbal_PID_calc(gimbal_PID_t *pid, float get, float set, float error_delta);

/**
 * @brief          ��̨PID��������pid��out,iout
 * @param[out]     pid_clear:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);

//��̨���������������
gimbal_control_t gimbal_control;

//���̿��������������
extern chassis_move_t chassis_move;

float gimbal_debug[10] = {0};
float gimbal_debug1[10] = {0};

//���͵ĵ������
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;

void gimbal_task(void const *pvParameters)
{
  vTaskDelay(GIMBAL_TASK_INIT_TIME);

  //��̨��ʼ��
  gimbal_init();

  //�жϵ���Ƿ�����
	
		while (toe_is_error( YAW_GIMBAL_MOTOR_TOE)||  toe_is_error(PITCH_GIMBAL_MOTOR_TOE)) 
		{
			vTaskDelay(GIMBAL_CONTROL_TIME);
			gimbal_feedback_update(); //��̨���ݷ���
		}

  while (1)
  {
    //������̨����ģʽ
    gimbal_set_mode();
    //��̨���ݷ���
    gimbal_feedback_update();
    //������̨������
    gimbal_set_control();
    //��̨����PID����
    gimbal_control_loop();
    //��̨��̬���ݷ��͵���λ��(����gimbal_behaviour��ʹ��)
    gimbal_data_send();
		

//��������װ�����෴����Ҫ�ڵ���ֵǰ��Ӹ���
#if YAW_TURN
    //yaw_can_set_current = -PID_deadline_limit(&gimbal_control.gimbal_yaw_motor.given_current,2000.0);
		yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
    yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
#endif
#if PITCH_TURN
    pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
    pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
#endif

    if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE)))
    {
      if (toe_is_error(DBUS_TOE))
      {
        CAN_cmd_gimbal(0, 0);
        //��ֹʧ��
        gimbal_control.gimbal_pitch_motor.absolute_angle_set = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle;
      }
      else
      {
        CAN_cmd_gimbal(yaw_can_set_current, pitch_can_set_current);
		
      }
    }
			

#if debug_mode
		
		float f1=gimbal_control.gimbal_pitch_motor.absolute_angle;
		float f2=gimbal_control.gimbal_pitch_motor.absolute_angle_set;
    if (toe_is_error(YAW_GIMBAL_MOTOR_TOE))
    {
      gimbal_debug[7] = 5;
    }
    else if (toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
    {
      gimbal_debug[7] = -5;
    }
    gimbal_debug[0] = gimbal_control.gimbal_pitch_motor.absolute_angle;
    gimbal_debug[1] = gimbal_control.gimbal_pitch_motor.absolute_angle_set;
    gimbal_debug[2] = gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.out;
    gimbal_debug[3] = gimbal_control.gimbal_pitch_motor.given_current;
    gimbal_debug[4] = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->given_current;
    gimbal_debug[5] = auto_shoot.pitch_add;
    gimbal_debug[6] = auto_shoot.yaw_add;
		
	  gimbal_debug1[0]=gimbal_control.gimbal_yaw_motor.absolute_angle;
		gimbal_debug1[1]=gimbal_control.gimbal_yaw_motor.absolute_angle_set;
		
#endif

    vTaskDelay(GIMBAL_CONTROL_TIME);
  }
}

/**
 * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
 * @retval         none
 */
static void gimbal_init(void)
{
  //�������ָ���ȡ
  gimbal_control.gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();                //��ȡ�����ǰ�Ƕ�
  gimbal_control.gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();           //��ȡ�����ǰ�Ƕ�
  //��ʼ�����ģʽ
  gimbal_control.gimbal_yaw_motor.gimbal_motor_mode = gimbal_control.gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;       // δ��ʼ��ģʽ
  gimbal_control.gimbal_pitch_motor.gimbal_motor_mode = gimbal_control.gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
  //��ʼ����̨��ֵ����λ
  gimbal_control.gimbal_yaw_motor.offset_ecd = GIMBAL_YAW_OFFSET_ECD;
  gimbal_control.gimbal_yaw_motor.max_relative_angle = GIMBAL_YAW_MAX_ECD;
  gimbal_control.gimbal_yaw_motor.min_relative_angle = GIMBAL_YAW_MIN_ECD;

  gimbal_control.gimbal_pitch_motor.offset_ecd = GIMBAL_PITCH_OFFSET_ECD;
  gimbal_control.gimbal_pitch_motor.max_relative_angle = GIMBAL_PITCH_MAX_ECD;
  gimbal_control.gimbal_pitch_motor.min_relative_angle = GIMBAL_PITCH_MIN_ECD;

  //��ʼ��yaw���pid
  gimbal_PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_ABSOLUTE_ANGLE_KP, YAW_ABSOLUTE_ANGLE_KI, YAW_ABSOLUTE_ANGLE_KD, YAW_ABSOLUTE_ANGLE_MAX_OUT, YAW_ABSOLUTE_ANGLE_MAX_IOUT); //�����ǽǶȿ���
  PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_GYRO_KP, YAW_GYRO_KI, YAW_GYRO_KD, YAW_GYRO_MAX_OUT, YAW_GYRO_MAX_IOUT);
  //��ʼ��pitch���pid
  gimbal_PID_init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_ABSOLUTE_ANGLE_KP, PITCH_ABSOLUTE_ANGLE_KI, PITCH_ABSOLUTE_ANGLE_KD, PITCH_ABSOLUTE_ANGLE_MAX_OUT, PITCH_ABSOLUTE_ANGLE_MAX_IOUT); //�����ǽǶȿ���
  PID_init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid, PITCH_GYRO_KP, PITCH_GYRO_KI, PITCH_GYRO_KD, PITCH_GYRO_MAX_OUT, PITCH_GYRO_MAX_IOUT);

  //�������PID���
  gimbal_total_pid_clear(&gimbal_control);
  //���ݸ���
  gimbal_feedback_update();
  //��ʼ������״̬
  gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle;
  gimbal_control.gimbal_yaw_motor.relative_angle_set = gimbal_control.gimbal_yaw_motor.relative_angle;
  gimbal_control.gimbal_yaw_motor.motor_gyro_set = gimbal_control.gimbal_yaw_motor.motor_gyro;

  gimbal_control.gimbal_pitch_motor.absolute_angle_set = gimbal_control.gimbal_pitch_motor.absolute_angle;
  gimbal_control.gimbal_pitch_motor.relative_angle_set = gimbal_control.gimbal_pitch_motor.relative_angle;
  gimbal_control.gimbal_pitch_motor.motor_gyro_set = gimbal_control.gimbal_pitch_motor.motor_gyro;

  //��ʼ����������ָ��
  user_send_data.gimbal_pitch_angle = &gimbal_control.gimbal_pitch_motor.absolute_angle;
  user_send_data.gimbal_yaw_gyro = &gimbal_control.gimbal_yaw_motor.motor_gyro;
	
}

/**
 * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
 * @retval         none
 */
static void gimbal_set_mode()
{
  gimbal_behaviour_mode_set(&gimbal_control);
}

/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @retval         none
 */
static void gimbal_feedback_update()
{
//��̨���ݸ���
//����������Ƕȱ仯����������ǽǶȱ仯�����෴����Ҫ����ԽǶ�ǰ��Ӹ���
#if PITCH_TURN
  gimbal_control.gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control.gimbal_pitch_motor.offset_ecd);
#else
  gimbal_control.gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control.gimbal_pitch_motor.offset_ecd);
#endif
  gimbal_control.gimbal_pitch_motor.absolute_angle = INS_data.angle_pitch;
  gimbal_control.gimbal_pitch_motor.motor_gyro = INS_data.wy;

#if YAW_TURN
  gimbal_control.gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control.gimbal_yaw_motor.offset_ecd);
#else
  gimbal_control.gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control.gimbal_yaw_motor.offset_ecd);
#endif
  gimbal_control.gimbal_yaw_motor.absolute_angle = INS_data.angle_yaw;
  gimbal_control.gimbal_yaw_motor.motor_gyro = -(arm_cos_f32(gimbal_control.gimbal_pitch_motor.relative_angle) * INS_data.wz - arm_sin_f32(gimbal_control.gimbal_pitch_motor.relative_angle) * INS_data.wx);
}

/**
 * @brief          ����ecd��offset_ecd֮�����ԽǶ�
 * @param[in]      ecd: �����ǰ����
 * @param[in]      offset_ecd: �����ֵ����
 * @retval         ��ԽǶȣ���λrad
 */

static float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
  int32_t relative_ecd = ecd - offset_ecd;
  if (relative_ecd > 4096)
  {
    relative_ecd -= 8191;
  }
  else if (relative_ecd < -4096)
  {
    relative_ecd += 8191; 
  }

  return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
 * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
 * @retval         none
 */
static void gimbal_set_control(void)
{
  float add_yaw_angle = 0.0f;
  float add_pitch_angle = 0.0f;

  gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, &gimbal_control);   // add_yaw_angle  add_pitch_angle   ң���������
  // yaw       
  gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle_set + add_yaw_angle;
  // pitch
  gimbal_absolute_angle_limit(&gimbal_control.gimbal_pitch_motor, add_pitch_angle); //���ԽǶȿ����޷�
}

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float add)
{
  static float bias_angle;
  static float angle_set;
  if (gimbal_motor == NULL)
  {
    return;
  }  //��ǰ�������Ƕ�
  bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
  //��̨��ԽǶ�+ ���Ƕ� + �����Ƕ� �����������е�Ƕ�
  if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)   //gimbal_motor->relative_angle  ����������
  {
    //�����������е�Ƕȿ��Ʒ���
    if (add > 0.0f)
    { 
      //�����һ��������ӽǶȣ�
      add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
    }
  }
  else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
  {
    if (add < 0.0f)
    {
      add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
    }
  }
  angle_set = gimbal_motor->absolute_angle_set;
  gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float add)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  gimbal_motor->relative_angle_set += add;
  //�Ƿ񳬹���� ��Сֵ
  if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
  {
    gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
  }
  else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
  {
    gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
  }
}

/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_control_loop()
{
	
	if(chassis_move.chassis_mode==CHASSIS_TOP)   //С����PID
	{
			gimbal_PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_ABSOLUTE_ANGLE_KP, YAW_ABSOLUTE_ANGLE_KI, -0.497f, YAW_ABSOLUTE_ANGLE_MAX_OUT, YAW_ABSOLUTE_ANGLE_MAX_IOUT); //�����ǽǶȿ���
      PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_GYRO_KP, YAW_GYRO_KI, YAW_GYRO_KD, YAW_GYRO_MAX_OUT, YAW_GYRO_MAX_IOUT);
	   
	}
	else                                         //����PID
	{
			gimbal_PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_ABSOLUTE_ANGLE_KP, YAW_ABSOLUTE_ANGLE_KI, YAW_ABSOLUTE_ANGLE_KD, YAW_ABSOLUTE_ANGLE_MAX_OUT, YAW_ABSOLUTE_ANGLE_MAX_IOUT); //�����ǽǶȿ���
      PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_GYRO_KP, YAW_GYRO_KI, YAW_GYRO_KD, YAW_GYRO_MAX_OUT, YAW_GYRO_MAX_IOUT);
	
	}
	// yaw
  if (gimbal_control.gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    gimbal_control.gimbal_yaw_motor.given_current = 0;
  }
  else
  { 
	 gimbal_motor_absolute_angle_control(&gimbal_control.gimbal_yaw_motor);
  }

  // pitch
  if (gimbal_control.gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    gimbal_control.gimbal_pitch_motor.given_current = 0;
  }
  else
  {
    gimbal_motor_absolute_angle_control(&gimbal_control.gimbal_pitch_motor);
  }
}

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
	
	//chassis_move_mode->chassis_mode ==CHASSIS_TOP
	
	
  //�ǶȻ����ٶȻ�����pid����
  gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);

  gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);//gimbal_motor->motor_gyro_set
  //����ֵ��ֵ
	gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);	  


}

/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  //�ǶȻ����ٶȻ�����pid����
  gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
  gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
  //����ֵ��ֵ
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
 * @brief ��̨PID��ʼ������
 *
 * @param pid PID���ݽṹָ��
 * @param kp
 * @param ki
 * @param kd
 * @param maxout ������
 * @param max_iout ���������
 */
static void gimbal_PID_init(gimbal_PID_t *pid, float kp, float ki, float kd, float maxout, float max_iout)
{
  if (pid == NULL)
  {
    return;
  }
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->err = 0.0f;
  pid->get = 0.0f;

  pid->max_iout = max_iout;
  pid->max_out = maxout;
}

/**
 * @brief ��̨PID���㺯��
 *
 * @param pid
 * @param get
 * @param set
 * @param error_delta ���΢���ֱ�ӴӴ�������ȡ��������
 * @return fp32
 */
static float gimbal_PID_calc(gimbal_PID_t *pid, float get, float set, float error_delta)
{
  float err;
  if (pid == NULL)
  {
    return 0.0f;
  }
  pid->get = get;
  pid->set = set;

  err = set - get;
  pid->err = rad_format(err);
  pid->Pout = pid->kp * pid->err;
  pid->Iout += pid->ki * pid->err;
  pid->Dout = pid->kd * error_delta;
  abs_limit(&pid->Iout, pid->max_iout);
  pid->out = pid->Pout + pid->Iout + pid->Dout;
  abs_limit(&pid->out, pid->max_out);
	//PID_deadline_limit(&pid->out,2.0);
  return pid->out;
}

/**
 * @brief          ��̨PID��������pid��out,iout
 * @param[out]     gimbal_pid_clear:"gimbal_control"����ָ��.
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
  if (gimbal_pid_clear == NULL)
  {
    return;
  }
  gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
  gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

/**
 * @brief          ����yaw �������ָ��
 * @param[in]      none
 * @retval         yaw���ָ��
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
  return &gimbal_control.gimbal_yaw_motor;
}

/**
 * @brief          ����pitch �������ָ��
 * @param[in]      none
 * @retval         pitch
 */
const gimbal_motor_t *get_pitch_motor_point(void)
{
  return &gimbal_control.gimbal_pitch_motor;
}

/**
 * @brief ��̨��̬���ݷ��͵���λ��
 *
 */
void gimbal_data_send(void)
{
  static uint8_t send_time;
  //ÿ5���뷢��һ��  500Hz
  if (send_time == 2)
  {
    user_data_pack_handle();
    send_time = 0;
  }
  send_time++;
}
