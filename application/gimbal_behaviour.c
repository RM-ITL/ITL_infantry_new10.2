/**
 * @file gimbal_behaviour.c
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
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "user_lib.h"
#include "USART_receive.h"
#include "config.h"
#include <math.h>

//����̨��У׼, ���÷�����Ƶ�ʺ�ǿ��
#define gimbal_warn_buzzer_on() buzzer_on(31, 20000)
#define gimbal_warn_buzzer_off() buzzer_off()

#define int_abs(x) ((x) > 0 ? (x) : (-x))

/**
 * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ��Ϊ0��
 * @param          �����ң����ֵ
 * @param          ��������������ң����ֵ
 * @param          ����ֵ
 */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
 * @brief          ��̨��Ϊ״̬������.
 * @param[in]      gimbal_mode_set: ��̨����ָ��
 * @retval         none
 */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);

/**
 * @brief          ����̨��Ϊģʽ��GIMBAL_ZERO_FORCE, ��������ᱻ����,��̨����ģʽ��rawģʽ.ԭʼģʽ��ζ��
 *                 �趨ֵ��ֱ�ӷ��͵�CAN������,�������������������Ϊ0.
 * @param[in]      yaw:����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
 * @param[in]      pitch:����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
 * @param[in]      gimbal_control_set: ��̨����ָ��
 * @retval         none
 */
static void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
 * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
 * @retval         none
 */
static void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
 * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set: ��̨����ָ��
 * @retval         none
 */
static void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
 * @author         RM
 * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
 * @retval         none
 */
static void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          ��̨��������ģʽ��ͨ����λ�����͵����ݿ���
 * @author         RM
 * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
 * @retval         none
 */
static void gimbal_auto_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

//����
static void gimbal_pole_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

//��̨��Ϊ״̬��
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
 * @brief          ��gimbal_set_mode����������gimbal_task.c,��̨��Ϊ״̬���Լ����״̬������
 * @param[out]     gimbal_mode_set: ��̨����ָ��
 * @retval         none
 */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //��̨��Ϊ״̬������
    gimbal_behavour_set(gimbal_mode_set);

    //������̨��Ϊ״̬�����õ��״̬��
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)                                       //����ģʽ
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)                                       // ��ʼģʽ
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)                             // ����ģʽ
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)									     				// ���ģʽ
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)                                 // ���˶�ģʽ
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
}

/**
 * @brief          ��̨��Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
 * @param[out]     add_yaw:���õ�yaw�Ƕ�����ֵ����λ rad
 * @param[out]     add_pitch:���õ�pitch�Ƕ�����ֵ����λ rad
 * @param[in]      gimbal_mode_set:��̨����ָ��
 * @retval         none
 */
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint8_t last_behaviour = GIMBAL_ZERO_FORCE;
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_AUTO)
    {
        gimbal_auto_control(add_yaw, add_pitch, gimbal_control_set);
    }
	else if (gimbal_behaviour == GIMBAL_POLE)
    {
        gimbal_pole_control(add_yaw, add_pitch, gimbal_control_set);
    }

    if (last_behaviour != gimbal_behaviour)
    {
        gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = gimbal_control_set->gimbal_pitch_motor.absolute_angle;
        gimbal_control_set->gimbal_yaw_motor.absolute_angle_set = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
    }

    last_behaviour = gimbal_behaviour;
}

/**
 * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���̲���
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���ֹͣ
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          ��̨��Ϊ״̬������.
 * @param[in]      gimbal_mode_set: ��̨����ָ��
 * @retval         none
 */
 int up=0,medium=0,down=0;
int leftkey = 0;
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
	  if (gimbal_mode_set == NULL)
    {
        return;
    }
		
		
    //���ؿ��� ��̨״̬(�Ǳ�Ҫ�������)
    if (switch_is_up(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL])) //��
    {
#ifdef AUTO_DEBUG
        gimbal_behaviour = GIMBAL_AUTO;
#else
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
#endif
    }
    else if (switch_is_mid(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL])) //��
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }
    else if (switch_is_down(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL])) //��
    {
#ifdef POLE_CONTRAL
        gimbal_behaviour = GIMBAL_POLE;
#else
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
#endif
    }
		
		if(up==1) gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;;
		if(medium==1) gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;;
		if(down==1) gimbal_behaviour = GIMBAL_ZERO_FORCE;
		
    if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z) {up=0;medium=1;down=0;}    //�����������
		
		/*       ���̿���       */
		
		
		if(rc_ctrl.key & KEY_PRESSED_OFFSET_F||rc_ctrl.rc.s[1]==3)     //��
		{
			gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
			
			if (rc_ctrl.rc.ch[4] > 0)
			{
				leftkey = 1;
			}
			else if (rc_ctrl.rc.ch[4] < 0)
			{
				leftkey = 0;
			}
		
			if(rc_ctrl.mouse.press_r!=0||leftkey==1)   //����Ҽ���������
			{
			  gimbal_behaviour = GIMBAL_AUTO;
			}
			
			up=1;
			down=0;
			medium=0;
		}
		
//		if(rc_ctrl.key & KEY_PRESSED_OFFSET_E||medium==1)    //��
//		{
//		  
//			
//			if(rc_ctrl.mouse.press_r!=0)   //����Ҽ���������
//			{
//			  gimbal_behaviour = GIMBAL_AUTO;
//			}
//			else 
//			{
//			   gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
//				 
//			}
//			
//			medium=1;
//			up=0;
//			down=0;
//		}
//		if(rc_ctrl.key & KEY_PRESSED_OFFSET_R)     //��
//		{
//		  gimbal_behaviour = GIMBAL_ZERO_FORCE;
//			down=1;
//			up=0;
//			medium=0;
//		}
		
		
		
    if (toe_is_error(DBUS_TOE))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
		
}

/**
 * @brief          ����̨��Ϊģʽ��GIMBAL_ZERO_FORCE, ��������ᱻ����,��̨����ģʽ��rawģʽ.ԭʼģʽ��ζ��
 *                 �趨ֵ��ֱ�ӷ��͵�CAN������,�������������������Ϊ0.
 * @param[in]      yaw:����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
 * @param[in]      pitch:����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
 * @param[in]      gimbal_control_set: ��̨����ָ��
 * @retval         none
 */
static void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
 * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
 * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
 * @retval         none
 */

 int turn_left_flag=0;
 int turn_right_flag=0;
 int turn_flag=0;
 int turn_up=0;
 int turn_down=0;

extern gimbal_control_t gimbal_control;

float anglex=0;
int lim_flag=0;
static int Turn=0;

int wkw[1];
int zx;
static float xishu1;
static float xishu2;

int shubiao;

static void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
   
//	  zx+=(rc_ctrl.mouse.z);
//	  wkw[0]=rc_ctrl.mouse.z;
	
	   shubiao=rc_ctrl.mouse.x;
	
	  if(rc_ctrl.mouse.press_r!=0)   //����Ҽ�΢��
			{
			  xishu1=YAW_MOUSE_SEN/3.5;
				xishu2=PITCH_MOUSE_SEN/3.0;
				
//				int flag1;
//				if(!rc_ctrl.key & KEY_PRESSED_OFFSET_B)
//				{flag1=2;}
//				if(rc_ctrl.key & KEY_PRESSED_OFFSET_B&&flag1)
//				{
//				  short_add=1;
//					flag1=0;
//				}
//				else
//				{
//				  short_add=0;
//				}
			}
			else
			{
			  xishu1=YAW_MOUSE_SEN;
				xishu2=PITCH_MOUSE_SEN;
				
//				int flag2;
//				if(!rc_ctrl.key & KEY_PRESSED_OFFSET_B)
//				{flag2=1;}
//				if(rc_ctrl.key & KEY_PRESSED_OFFSET_B&&flag2)
//				{
//				  short_minus=2;
//					flag2=0;
//				}
//				else
//				{
//				  short_minus=0;
//				}
			}
			
	
	  if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(rc_ctrl.rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(rc_ctrl.rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
		
		int wk=1;
		int sage=1;
	
//		if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT))     //�����������
//   {lim_flag=1;}
//	 if((rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)&&lim_flag)
//	{
//		lim_flag=0;
//		if(Turn==0) Turn=1;
//		else Turn=0;
//		
//	}
//	  if(Turn)      //�������
//		{
//			if(rc_ctrl.mouse.x<=2&&rc_ctrl.mouse.x>=-2) wk=0;
//			if(rc_ctrl.mouse.y<=5&&rc_ctrl.mouse.y>=-5) sage=0;
//		}
		
    *yaw = yaw_channel * YAW_RC_SEN - rc_ctrl.mouse.x * xishu1;
    *pitch = pitch_channel * PITCH_RC_SEN + (rc_ctrl.mouse.y * xishu2-rc_ctrl.mouse.z*xishu2*0.6f);
		
if(1)   //�Ľ�
{
		
	/******************************��ͷ*********************************/		
	if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_R))
   {turn_flag=1;} 
	 if((rc_ctrl.key & KEY_PRESSED_OFFSET_R)&&turn_flag)
	{
		turn_flag=0;
		*yaw=*yaw+PI;
	}
	/******************************��ת90��*********************************/		
	if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_Q))
   {turn_left_flag=1;} 
	 if((rc_ctrl.key & KEY_PRESSED_OFFSET_Q)&&turn_left_flag)
	{
		turn_left_flag=0;
		*yaw=*yaw+PI/2;
	}
	/******************************��ת90��*********************************/		
	if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_E))
   {turn_right_flag=1;} 
	 if((rc_ctrl.key & KEY_PRESSED_OFFSET_E)&&turn_right_flag)
	{
		turn_right_flag=0;
		*yaw=*yaw-PI/2;
	}
}	
else    //  ����
{
		
		if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z)  
      {turn_left_flag=0;turn_right_flag=0;turn_flag=0;};     //������������
			
			
	/******************************��ͷ*********************************/	
			
		if (rc_ctrl.key & KEY_PRESSED_OFFSET_R || turn_flag==1)         
  {
    
		if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z) //�����������
		{turn_flag=0;turn_left_flag=0;turn_right_flag=0;}
		if (turn_flag== 0)
		{ anglex= gimbal_control.gimbal_yaw_motor.absolute_angle_set;}
     
    turn_flag = 1;
		
    if (anglex >= 0)
    {
      gimbal_control.gimbal_yaw_motor.absolute_angle_set = anglex - PI;
    }
    else
    {
      gimbal_control.gimbal_yaw_motor.absolute_angle_set = anglex + PI;
    }
    if (fabs(gimbal_control.gimbal_yaw_motor.absolute_angle_set - gimbal_control.gimbal_yaw_motor.absolute_angle) <= 0.1)
      turn_flag = 0;
  }
	
	
	
		/******************************��ת90��*********************************/		
	/*
	
	      0 
	
pi/2          -pi/2	
	
	    pi -pi
	
	*/
		if (rc_ctrl.key & KEY_PRESSED_OFFSET_E || turn_right_flag==1)         
  {
		
		if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z) //�����������
		{turn_flag=0;turn_left_flag=0;turn_right_flag=0;}
		
    if(turn_right_flag==0)
		{anglex=gimbal_control.gimbal_yaw_motor.absolute_angle_set;}
		
    turn_right_flag = 1;
		
		
		if(anglex>=-PI/2)
     {gimbal_control.gimbal_yaw_motor.absolute_angle_set = anglex - PI/2;}
		else
     {gimbal_control.gimbal_yaw_motor.absolute_angle_set = anglex + PI*3.0/2;}
    
  
		
    if (fabs(gimbal_control.gimbal_yaw_motor.absolute_angle_set - gimbal_control.gimbal_yaw_motor.absolute_angle) <= 0.1)
      turn_right_flag = 0;
  }
	
	
	
		/******************************��ת90��*********************************/	

	/*
	
	      0 
	
pi/2          -pi/2	
	
	      pi
	
	*/	
		if (rc_ctrl.key & KEY_PRESSED_OFFSET_Q || turn_left_flag==1)         
  {
    if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z)     //�����������
		{turn_flag=0;turn_left_flag=0;turn_right_flag=0;}
		
		if(turn_left_flag == 0)
		{ anglex = gimbal_control.gimbal_yaw_motor.absolute_angle_set;}
		
    turn_left_flag = 1;
		
		if(anglex>=PI/2) 
      {gimbal_control.gimbal_yaw_motor.absolute_angle_set = anglex-3.0/2*PI;}
		else 
      {gimbal_control.gimbal_yaw_motor.absolute_angle_set = anglex+PI/2;}
		
    if (fabs(gimbal_control.gimbal_yaw_motor.absolute_angle_set - gimbal_control.gimbal_yaw_motor.absolute_angle) <= 0.1)
      turn_left_flag = 0;
  }
	
	
//	
//		
//		/*   ��ת90��   */
//		if(rc_ctrl.key & KEY_PRESSED_OFFSET_C)
//		{
//		  turn_left_flag=1;
//			anglex=gimbal_control.gimbal_yaw_motor.absolute_angle+PI/2;
//		}
//		
//		if(turn_left_flag==1) 
//		{
//		   *yaw =PI/700+(yaw_channel * YAW_RC_SEN - rc_ctrl.mouse.x * YAW_MOUSE_SEN);
//			
//      if(rc_ctrl.key & KEY_PRESSED_OFFSET_X) turn_left_flag=0;      //��ֹ��ת
//			if(fabs(gimbal_control.gimbal_yaw_motor.absolute_angle-anglex)<=PI/700*12)
//				turn_left_flag=0;
//		
//		}
//		
//		/*   ��ת90��  */
//				if(rc_ctrl.key & KEY_PRESSED_OFFSET_V)
//		{
//		  turn_right_flag=1;
//			anglex1=gimbal_control.gimbal_yaw_motor.absolute_angle-PI/2;
//		}
//		
//		if(turn_right_flag==1) 
//		{
//		   *yaw =PI/700+(yaw_channel * YAW_RC_SEN - rc_ctrl.mouse.x * YAW_MOUSE_SEN);
//			if(rc_ctrl.key & KEY_PRESSED_OFFSET_X) turn_right_flag=0;    //��ֹ��ת
//			if(fabs(gimbal_control.gimbal_yaw_motor.absolute_angle-anglex1)<=PI/700*12)
//				turn_right_flag=0;
//		
//		}
//		/*  ��ͷ  */
//						if(rc_ctrl.key & KEY_PRESSED_OFFSET_B)
//		{
//		  turn_flag=1;
//			anglex2=gimbal_control.gimbal_yaw_motor.absolute_angle-PI;
//		}
//		
//		if(turn_flag==1) 
//		{
//		   *yaw =-PI/400+(yaw_channel * YAW_RC_SEN - rc_ctrl.mouse.x * YAW_MOUSE_SEN);
//			if(rc_ctrl.key & KEY_PRESSED_OFFSET_X) turn_flag=0;       //��ֹ��ͷ
//			if(fabs(gimbal_control.gimbal_yaw_motor.absolute_angle-anglex2)<=PI/400*15)
//				turn_flag=0;
//		
//		}
//		
		
		
}
		
}

/**
 * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
 * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set: ��̨����ָ��
 * @retval         none
 */
static void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(rc_ctrl.rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(rc_ctrl.rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw = yaw_channel * YAW_RC_SEN - rc_ctrl.mouse.x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + rc_ctrl.mouse.y * PITCH_MOUSE_SEN;
//		gimbal_data_send();
}

/**
 * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
 * @author         RM
 * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
 * @param[in]      gimbal_control_set:��̨����ָ��
 * @retval         none
 */
static void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

static void gimbal_auto_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)//����
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    //����λ��������̬����
//    gimbal_data_send();

    // ����Ƿ��ܵ���λ�������ź�
    if (toe_is_error(USER_USART_DATA_TOE))
    {
        *yaw = 0.0f;
        *pitch = 0.0f;
    }
    else
    {
        *yaw = 0.001f*auto_shoot.yaw_add;
        *pitch = 0.001f*auto_shoot.pitch_add;
    }
}

static void gimbal_pole_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)//����΢��
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    //����λ��������̬����
    gimbal_data_send();

    // ����Ƿ��ܵ���λ�������ź�
    if (toe_is_error(USER_USART_DATA_TOE))
    {
        *yaw = 0.0f;
        *pitch = 0.0f;
    }
	
	static int16_t pitch_channel = 0;
	
	float  a = PITCH_MOUSE_SEN;
	
	rc_deadband_limit(rc_ctrl.rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
	
	*pitch = pitch_channel * PITCH_RC_SEN + (rc_ctrl.mouse.y * a - rc_ctrl.mouse.z* a *0.6f);
	
		/****************************** ����˿����΢�� *********************************/	
	if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_X))
   {turn_up=1;} 
	 if((rc_ctrl.key & KEY_PRESSED_OFFSET_X)&&turn_up)
	{
		turn_up=0;
		*pitch=*pitch+PI/36;
	}		
	/****************************** ����˿����΢�� *********************************/	
	if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_C))
   {turn_down=1;} 
	 if((rc_ctrl.key & KEY_PRESSED_OFFSET_C)&&turn_down)
	{
		turn_down=0;
		*pitch=*pitch-PI/36;
	}
}