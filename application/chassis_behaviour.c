/**
 * @file chassis_behaviour.c
 * @author ���廪
 * @brief
 * @version 0.1
 * @date 2022-03-17
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
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "config.h"

#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }

extern void chassis_power_control_update(void);
	
	
/**
 * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_zero_force_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_no_move_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_follow_gimbal_yaw_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ����С���ݵ���Ϊ״̬���£�����ģʽ��һ����תһ������ָ̨�����˶�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_top_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_no_follow_yaw_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         none
 */
static void chassis_open_set_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
 * @param[out]     vx_set: �����ٶ�ָ��
 * @param[out]     vy_set: �����ٶ�ָ��
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
 * @retval         none
 */
extern void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ����ģʽѡ��
 * @param[in]      chassis_move_mode: ��������
 * @retval         none
 */
 int up2=0,medium2=0,down2=0;
int untop_flag=0;
int unfollow_flag=0;
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
	
	if (chassis_move_mode == NULL)
  {
    return;
  }
	

  //ң��������ģʽ(�Ǳ�Ҫ�������)
  if (switch_is_up(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) //��
  {
#ifdef AUTO_DEBUG
    chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
#else
    chassis_move_mode->chassis_mode = CHASSIS_TOP;
#endif
  }
  else if (switch_is_mid(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) //��
  {
    chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
  }
  else if (switch_is_down(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) //��
  {
    chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL;
  }
	
	if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z) //�����������
    {up2=0;medium2=1;down2=0;}    
		
	if(up2==1)      chassis_move_mode->chassis_mode = CHASSIS_TOP;
	if(medium2==1)  chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
	if(down2==1)    chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL;
		
	

	                              /*       ���̿���       */
		
	/**********************************С����ģʽ(��)***************************************/		

if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_F) )//F��С����
   {untop_flag=1;}
	 
	if((rc_ctrl.key & KEY_PRESSED_OFFSET_F)&&untop_flag)     //С����ģʽ   (��) 
	{
		  untop_flag=0;
		
			if(up2==1)
			{
			   up2=0;
			   down2=0;
			   medium2=1;
			}
		else
		{			
			
			up2=1;
			down2=0;
			medium2=0;
			
		}
	}

	/**********************************����ģʽ(��)***************************************/
	
//	if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_G) )
//   {unfollow_flag=1;}	
//	
// 	if((rc_ctrl.key & KEY_PRESSED_OFFSET_G)&&unfollow_flag)     //����ģʽ(��)   G����̨������
//	{
//		  unfollow_flag=0;
//		
//			if(down2==1)
//			{
//			   up2=0;
//			   down2=0;
//			   medium2=1;
//			}
//		  else
//		{			
//			up2=0;
//			down2=1;
//			medium2=0;
//			
//		}
//	}
//	
		if(rc_ctrl.key & KEY_PRESSED_OFFSET_C)     
		{
		  chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;//C����̨����
			down2=0;
			up2=0;
			medium2=1;
		}
		
	
  //����̨��ĳЩģʽ�£����ʼ���� ���̲���
  if (gimbal_cmd_to_chassis_stop())
  {
    chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;
  }
}

/**
 * @brief          ���ÿ�����.���ݲ�ͬ���̿���ģʽ������ò�ͬ�Ŀ��ƺ���.
 * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
 * @retval         none
 */
void chassis_behaviour_control_set(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_MOVE)
  {
    chassis_no_move_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_FOLLOW_GIMBAL)
  {
    chassis_follow_gimbal_yaw_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_TOP)
  {
    chassis_top_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_FOLLOW_GIMBAL)
  {
    chassis_no_follow_yaw_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_OPEN)
  {
    chassis_open_set_control(chassis_move_rc_to_vector);
  }
  if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_ZERO_FORCE)
  {
    chassis_zero_force_control(chassis_move_rc_to_vector);

  }
}

/**
 * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_no_move_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  chassis_move_rc_to_vector->vx_set = 0.0f;
  chassis_move_rc_to_vector->vy_set = 0.0f;
  chassis_move_rc_to_vector->wz_set = 0.0f;
}

/**
 * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕ�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_follow_gimbal_yaw_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  float vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;

  //ң������ͨ��ֵ�Լ����̰��� �ó� һ������µ��ٶ��趨ֵ
  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);
  float sin_yaw = 0.0f, cos_yaw = 0.0f;
	int xishu=0;


	/////////////////////////��ά������ת�㷨-�����ݸ��ѧԺ2024.02.29/////////////////////////////////////
	{
		float SpinTop_Angle=0;
		SpinTop_Angle  = (gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd+1200+2048 + 227+227) % 8192 / 22.7555556f;//22.7555556f=8192/360,����ֵתΪ�Ƕ�ֵ
		if(SpinTop_Angle > 360)
			SpinTop_Angle = (SpinTop_Angle - 360) * 0.0174532f;//2*pi/360 �Ƕ�ֵת����ֵ
		else
			SpinTop_Angle *= 0.0174532f;

		chassis_move_rc_to_vector->vx_set = vx_set * cos(SpinTop_Angle) + vy_set * sin(SpinTop_Angle);
		chassis_move_rc_to_vector->vy_set = -vx_set * sin(SpinTop_Angle) + vy_set * cos(SpinTop_Angle);
	}
	
	
  //���ÿ��������̨�Ƕ�
  chassis_move_rc_to_vector->chassis_relative_angle_set = rad_format(angle_set)-2.33f + 3.14f + 0.174f;//-0.405f;//2.8//-1.23f;//+0.4f;//+0.125f;//2024.1.9:0.125f������̨ƫ��
  chassis_move_rc_to_vector->chassis_relative_angle = rad_format(chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle - GIMBAL_YAW_OFFSET_ECD);
  //������תPID���ٶ�
  chassis_move_rc_to_vector->wz_set = PID_calc(&chassis_move_rc_to_vector->chassis_angle_pid, chassis_move_rc_to_vector->chassis_relative_angle, chassis_move_rc_to_vector->chassis_relative_angle_set);

  //�ٶ��޷�
  chassis_move_rc_to_vector->wz_set = float_constrain(chassis_move_rc_to_vector->wz_set, -10.0f, 10.0f);//6.0
  chassis_move_rc_to_vector->vx_set = float_constrain(chassis_move_rc_to_vector->vx_set, chassis_move_rc_to_vector->vx_min_speed, chassis_move_rc_to_vector->vx_max_speed);//chassis_move_rc_to_vector->vx_min_speed//chassis_move_rc_to_vector->vx_max_speed
  chassis_move_rc_to_vector->vy_set = float_constrain(chassis_move_rc_to_vector->vy_set, chassis_move_rc_to_vector->vy_min_speed, chassis_move_rc_to_vector->vy_max_speed);//chassis_move_rc_to_vector->vy_min_speed//chassis_move_rc_to_vector->vy_max_speed
}

/**
 * @brief          ����С���ݵ���Ϊ״̬���£�����ģʽ��һ����תһ������ָ̨�����˶�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */

int flagx1;
int v=CHASSIS_TOP_SPEED;

static void chassis_top_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  float vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;

  //ң������ͨ��ֵ�Լ����̰��� �ó� һ������µ��ٶ��趨ֵ
  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);
  float sin_yaw = 0.0f, cos_yaw = 0.0f;


	
	 /////////////////////////��ά������ת�㷨-�����ݸ��ѧԺ2024.02.29/////////////////////////////////////
	{
		float SpinTop_Angle=0;
		SpinTop_Angle  = (gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd - GIMBAL_YAW_OFFSET_ECD + 8192) % 8192 / 22.7555556f;//22.7555556f=8192/360,����ֵתΪ�Ƕ�ֵ
		if(SpinTop_Angle > 360)
			SpinTop_Angle = (SpinTop_Angle - 360) * 0.0174532f;//2*pi/360
		else
			SpinTop_Angle *= 0.0174532f;

		chassis_move_rc_to_vector->vx_set = vx_set * cos(SpinTop_Angle) - vy_set * sin(SpinTop_Angle);
		chassis_move_rc_to_vector->vy_set = vx_set * sin(SpinTop_Angle) + vy_set * cos(SpinTop_Angle);
	}
	
	
		/****С����ģʽѡ��****/
	if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_V))//V����С����
   {flagx1=1;} 
	 if((rc_ctrl.key & KEY_PRESSED_OFFSET_V)&&flagx1)
	{
		flagx1=0;
		if(v==CHASSIS_TOP_SPEED)  v=10;    //����
		else v=CHASSIS_TOP_SPEED;    //����

	}
	
	
	
  //����С����ת��
  if (vx_set == 0 && vy_set == 0)
  {
    chassis_move_rc_to_vector->wz_set = v;
  }
  else
  {
    //ƽ��ʱС�����ٶȱ���
    chassis_move_rc_to_vector->wz_set = 1.2* CHASSIS_TOP_SPEED;  //���ƹ���,0.5������
  }
	

  //�ٶ��޷���С����ʱ�ƶ��ٶȱ�����
  chassis_move_rc_to_vector->vx_set = float_constrain(chassis_move_rc_to_vector->vx_set, chassis_move_rc_to_vector->top_min_speed, chassis_move_rc_to_vector->top_max_speed);
  chassis_move_rc_to_vector->vy_set = float_constrain(chassis_move_rc_to_vector->vy_set, chassis_move_rc_to_vector->top_min_speed, chassis_move_rc_to_vector->top_max_speed);
}
/**
 * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_no_follow_yaw_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  float vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);
  wz_set = -CHASSIS_WZ_RC_SEN * rc_ctrl.rc.ch[CHASSIS_WZ_CHANNEL];

  chassis_move_rc_to_vector->wz_set = wz_set;
  chassis_move_rc_to_vector->vx_set = float_constrain(vx_set, chassis_move_rc_to_vector->vx_min_speed, chassis_move_rc_to_vector->vx_max_speed);
  chassis_move_rc_to_vector->vy_set = float_constrain(vy_set, chassis_move_rc_to_vector->vy_min_speed, chassis_move_rc_to_vector->vy_max_speed);
}

/**
 * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         none
 */
static void chassis_open_set_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  chassis_move_rc_to_vector->vx_set = rc_ctrl.rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  chassis_move_rc_to_vector->vy_set = -rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  chassis_move_rc_to_vector->wz_set = -rc_ctrl.rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  return;

}

/**
 * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_zero_force_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  chassis_move_rc_to_vector->vx_set = 0.0f;
  chassis_move_rc_to_vector->vy_set = 0.0f;
  chassis_move_rc_to_vector->wz_set = 0.0f;
}

/**
 * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
 *
 * @param[out]     vx_set: �����ٶ�ָ��
 * @param[out]     vy_set: �����ٶ�ָ��
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
 * @retval         none
 */
void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
  {
    return;
  }

  int16_t vx_channel, vy_channel;
  float vx_set_channel, vy_set_channel;

  //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
  rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

  vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
  vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

  //���̿���
  if (rc_ctrl.key & KEY_PRESSED_OFFSET_W)
  {
    vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
		
  }
  else if (rc_ctrl.key & KEY_PRESSED_OFFSET_S)
  {
    vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
  }

  if (rc_ctrl.key & KEY_PRESSED_OFFSET_A)
  {
    vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
  }
  else if (rc_ctrl.key & KEY_PRESSED_OFFSET_D)
  {
    vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
  }

  // // //һ�׵�ͨ�˲�����б����Ϊ�����ٶ����루ƽ����
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
  //�������ڣ�ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
  if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
  }

  if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
  }

  // *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
  // *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
  *vx_set = vx_set_channel;
  *vy_set = vy_set_channel;
}



////��������
////int flag1 = 1;
////void chassis_power_control_update(void)
////	{
////if (!(rc_ctrl.key & KEY_PRESSED_OFFSET_Q)) 
////  {
////    flag1 = 1;
////  }
////  if ((rc_ctrl.key & KEY_PRESSED_OFFSET_Q) && flag1 ) 
////  {
////    flag1 = !flag1;

////  }
//// if (flag1 == 1)
////  {
////    chassis_power_control_update();
////  }
////}