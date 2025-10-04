/**
 * @file gimbal_behaviour.c
 * @author 何清华
 * @brief
 * @version 0.1
 * @date 2022-03-17
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */

/*
***********************某虽不才，即断对面之头，悬之东门，以谢天下***********************
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
												
***********************某虽不才，即断对面之头，悬之东门，以谢天下***********************

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

//当云台在校准, 设置蜂鸣器频率和强度
#define gimbal_warn_buzzer_on() buzzer_on(31, 20000)
#define gimbal_warn_buzzer_off() buzzer_off()

#define int_abs(x) ((x) > 0 ? (x) : (-x))

/**
 * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
 * @param          输入的遥控器值
 * @param          输出的死区处理后遥控器值
 * @param          死区值
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
 * @brief          云台行为状态机设置.
 * @param[in]      gimbal_mode_set: 云台数据指针
 * @retval         none
 */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);

/**
 * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
 *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
 * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
 * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          云台编码值控制，电机是相对角度控制，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
 * @author         RM
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          云台进入自瞄模式，通过上位机发送的数据控制
 * @author         RM
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_auto_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

//吊射
static void gimbal_pole_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);

//云台行为状态机
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
 * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
 * @param[out]     gimbal_mode_set: 云台数据指针
 * @retval         none
 */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //云台行为状态机设置
    gimbal_behavour_set(gimbal_mode_set);

    //根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)                                       //无力模式
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)                                       // 初始模式
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)                             // 绝对模式
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)									     				// 相对模式
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)                                 // 无运动模式
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
}

/**
 * @brief          云台行为控制，根据不同行为采用不同控制函数
 * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
 * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
 * @param[in]      gimbal_mode_set:云台数据指针
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
 * @brief          云台在某些行为下，需要底盘不动
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
 * @brief          云台在某些行为下，需要射击停止
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
 * @brief          云台行为状态机设置.
 * @param[in]      gimbal_mode_set: 云台数据指针
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
		
		
    //开关控制 云台状态(非必要不建议改)
    if (switch_is_up(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL])) //上
    {
#ifdef AUTO_DEBUG
        gimbal_behaviour = GIMBAL_AUTO;
#else
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
#endif
    }
    else if (switch_is_mid(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL])) //中
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }
    else if (switch_is_down(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL])) //下
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
		
    if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z) {up=0;medium=1;down=0;}    //键盘数据清空
		
		/*       键盘控制       */
		
		
		if(rc_ctrl.key & KEY_PRESSED_OFFSET_F||rc_ctrl.rc.s[1]==3)     //上
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
		
			if(rc_ctrl.mouse.press_r!=0||leftkey==1)   //鼠标右键开启自瞄
			{
			  gimbal_behaviour = GIMBAL_AUTO;
			}
			
			up=1;
			down=0;
			medium=0;
		}
		
//		if(rc_ctrl.key & KEY_PRESSED_OFFSET_E||medium==1)    //中
//		{
//		  
//			
//			if(rc_ctrl.mouse.press_r!=0)   //鼠标右键开启自瞄
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
//		if(rc_ctrl.key & KEY_PRESSED_OFFSET_R)     //下
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
 * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
 *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
 * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
 * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
 * @param[in]      gimbal_control_set: 云台数据指针
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
 * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
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
	
	  if(rc_ctrl.mouse.press_r!=0)   //鼠标右键微调
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
	
//		if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT))     //鼠标死区开关
//   {lim_flag=1;}
//	 if((rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)&&lim_flag)
//	{
//		lim_flag=0;
//		if(Turn==0) Turn=1;
//		else Turn=0;
//		
//	}
//	  if(Turn)      //鼠标死区
//		{
//			if(rc_ctrl.mouse.x<=2&&rc_ctrl.mouse.x>=-2) wk=0;
//			if(rc_ctrl.mouse.y<=5&&rc_ctrl.mouse.y>=-5) sage=0;
//		}
		
    *yaw = yaw_channel * YAW_RC_SEN - rc_ctrl.mouse.x * xishu1;
    *pitch = pitch_channel * PITCH_RC_SEN + (rc_ctrl.mouse.y * xishu2-rc_ctrl.mouse.z*xishu2*0.6f);
		
if(1)   //改进
{
		
	/******************************掉头*********************************/		
	if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_R))
   {turn_flag=1;} 
	 if((rc_ctrl.key & KEY_PRESSED_OFFSET_R)&&turn_flag)
	{
		turn_flag=0;
		*yaw=*yaw+PI;
	}
	/******************************左转90°*********************************/		
	if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_Q))
   {turn_left_flag=1;} 
	 if((rc_ctrl.key & KEY_PRESSED_OFFSET_Q)&&turn_left_flag)
	{
		turn_left_flag=0;
		*yaw=*yaw+PI/2;
	}
	/******************************右转90°*********************************/		
	if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_E))
   {turn_right_flag=1;} 
	 if((rc_ctrl.key & KEY_PRESSED_OFFSET_E)&&turn_right_flag)
	{
		turn_right_flag=0;
		*yaw=*yaw-PI/2;
	}
}	
else    //  初代
{
		
		if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z)  
      {turn_left_flag=0;turn_right_flag=0;turn_flag=0;};     //键盘数据清零
			
			
	/******************************掉头*********************************/	
			
		if (rc_ctrl.key & KEY_PRESSED_OFFSET_R || turn_flag==1)         
  {
    
		if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z) //键盘数据清空
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
	
	
	
		/******************************右转90°*********************************/		
	/*
	
	      0 
	
pi/2          -pi/2	
	
	    pi -pi
	
	*/
		if (rc_ctrl.key & KEY_PRESSED_OFFSET_E || turn_right_flag==1)         
  {
		
		if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z) //键盘数据清空
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
	
	
	
		/******************************左转90°*********************************/	

	/*
	
	      0 
	
pi/2          -pi/2	
	
	      pi
	
	*/	
		if (rc_ctrl.key & KEY_PRESSED_OFFSET_Q || turn_left_flag==1)         
  {
    if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z)     //键盘数据清空
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
//		/*   左转90°   */
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
//      if(rc_ctrl.key & KEY_PRESSED_OFFSET_X) turn_left_flag=0;      //终止左转
//			if(fabs(gimbal_control.gimbal_yaw_motor.absolute_angle-anglex)<=PI/700*12)
//				turn_left_flag=0;
//		
//		}
//		
//		/*   右转90°  */
//				if(rc_ctrl.key & KEY_PRESSED_OFFSET_V)
//		{
//		  turn_right_flag=1;
//			anglex1=gimbal_control.gimbal_yaw_motor.absolute_angle-PI/2;
//		}
//		
//		if(turn_right_flag==1) 
//		{
//		   *yaw =PI/700+(yaw_channel * YAW_RC_SEN - rc_ctrl.mouse.x * YAW_MOUSE_SEN);
//			if(rc_ctrl.key & KEY_PRESSED_OFFSET_X) turn_right_flag=0;    //终止右转
//			if(fabs(gimbal_control.gimbal_yaw_motor.absolute_angle-anglex1)<=PI/700*12)
//				turn_right_flag=0;
//		
//		}
//		/*  掉头  */
//						if(rc_ctrl.key & KEY_PRESSED_OFFSET_B)
//		{
//		  turn_flag=1;
//			anglex2=gimbal_control.gimbal_yaw_motor.absolute_angle-PI;
//		}
//		
//		if(turn_flag==1) 
//		{
//		   *yaw =-PI/400+(yaw_channel * YAW_RC_SEN - rc_ctrl.mouse.x * YAW_MOUSE_SEN);
//			if(rc_ctrl.key & KEY_PRESSED_OFFSET_X) turn_flag=0;       //终止掉头
//			if(fabs(gimbal_control.gimbal_yaw_motor.absolute_angle-anglex2)<=PI/400*15)
//				turn_flag=0;
//		
//		}
//		
		
		
}
		
}

/**
 * @brief          云台编码值控制，电机是相对角度控制，
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set: 云台数据指针
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
 * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
 * @author         RM
 * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
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

static void gimbal_auto_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)//自瞄
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    //向上位机发送姿态数据
//    gimbal_data_send();

    // 检测是否受到上位机控制信号
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

static void gimbal_pole_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)//吊射微调
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    //向上位机发送姿态数据
    gimbal_data_send();

    // 检测是否受到上位机控制信号
    if (toe_is_error(USER_USART_DATA_TOE))
    {
        *yaw = 0.0f;
        *pitch = 0.0f;
    }
	
	static int16_t pitch_channel = 0;
	
	float  a = PITCH_MOUSE_SEN;
	
	rc_deadband_limit(rc_ctrl.rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
	
	*pitch = pitch_channel * PITCH_RC_SEN + (rc_ctrl.mouse.y * a - rc_ctrl.mouse.z* a *0.6f);
	
		/****************************** 吊射丝杆上微调 *********************************/	
	if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_X))
   {turn_up=1;} 
	 if((rc_ctrl.key & KEY_PRESSED_OFFSET_X)&&turn_up)
	{
		turn_up=0;
		*pitch=*pitch+PI/36;
	}		
	/****************************** 吊射丝杆下微调 *********************************/	
	if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_C))
   {turn_down=1;} 
	 if((rc_ctrl.key & KEY_PRESSED_OFFSET_C)&&turn_down)
	{
		turn_down=0;
		*pitch=*pitch-PI/36;
	}
}