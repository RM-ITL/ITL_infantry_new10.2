
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             只控制80w功率，主要通过控制电机电流设定值,如果限制功率是40w，减少
  *             JUDGE_TOTAL_CURRENT_LIMIT和POWER_CURRENT_LIMIT的值，还有底盘最大速度
  *             (包括max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"

extern ext_game_robot_state_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;
#define POWER_LIMIT         70.0f
#define WARNING_POWER       50.0f     //40

#define POWER_LIMIT1         90.0f
#define WARNING_POWER1       70.0f  

#define POWER_LIMIT2         110.0f
#define WARNING_POWER2       90.0f 

#define WARNING_POWER_BUFF  45.0f  
#define WARNING_POWER_BUFF1  60.0f 
#define WARNING_POWER_BUFF2  80.0f 

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 

#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f

/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
	
/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */

 float buffer_total_current_limit=16000;//最大缓冲功率总电流限制
 float power_total_current_limit=20000;//最大功率总电流限制

 float power_limit= 30.0f;            //最大功率限制
 float wanning_power;          				//最大警告功率
 float wanning_power_buff;     				//最大警告缓冲能量
 
 float volt=0.0f;
 float current=0.0f;
 float chassis_power = 0.0f;   				//底盘功率
 float chassis_power_buffer = 0.0f;   //底盘缓冲能量
 
 float total_current_limit = 0.0f;
 float total_current = 0.0f;
 float total_current1 = 0.0f;
 float k;
 
 float power_scale;                    //缩小限度
 
 
 float limit_k=1.0f;
 float powerbuffer_const = 0.0f;
	
void chassis_power_control(chassis_move_t *chassis_power_control)
{
	total_current = 0.0f;
	total_current1=0.0f;
    //calculate the original motor current set
    //计算原本电机电流设定
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
    }
   if(robot_state.robot_level == 0) //机器人等级为0时无功率限制
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;//64000
    }
    else                        
    {
      
//			total_current=total_current+500;
//		  total_current_limit = (power_limit/volt);
//			total_current_limit=total_current_limit*1000000*0.504;//1.26
//			if (chassis_power_buffer > 60.0f && chassis_power_buffer <= 250.0f)
//			    {powerbuffer_const = chassis_power_buffer;}//算出来limit_k=1
//      else
//					{ powerbuffer_const = 60.0f;}

//        limit_k = 0.25f * ((float)chassis_power_buffer / powerbuffer_const) * ((float)chassis_power_buffer / powerbuffer_const) *
//                      ((float)chassis_power_buffer / powerbuffer_const) +
//                  0.5f * ((float)chassis_power_buffer / powerbuffer_const) *
//                      ((float)chassis_power_buffer / powerbuffer_const) +
//                  0.25f * ((float)chassis_power_buffer / powerbuffer_const);
			get_chassis_power_and_buffer(&current,&volt,&chassis_power, &chassis_power_buffer);
			if(chassis_power_buffer<50&&chassis_power_buffer>=40)	limit_k=0.5;//15
			
			else if(chassis_power_buffer<40&&chassis_power_buffer>=35)	limit_k=0.5;//0.75
			else if(chassis_power_buffer<35&&chassis_power_buffer>=30)	limit_k=0.5;
			else if(chassis_power_buffer<30&&chassis_power_buffer>=20)	limit_k=0.25;
			else if(chassis_power_buffer<20&&chassis_power_buffer>=10)	limit_k=0.125;
			else if(chassis_power_buffer<10&&chassis_power_buffer>=0)	limit_k=0.05;
			else if(chassis_power_buffer>=60)					limit_k=1;
			chassis_power_control->motor_speed_pid[0].out*=limit_k;
      chassis_power_control->motor_speed_pid[1].out*=limit_k;
      chassis_power_control->motor_speed_pid[2].out*=limit_k;
      chassis_power_control->motor_speed_pid[3].out*=limit_k;
			for(uint8_t i = 0; i < 4; i++)
			{
        total_current1 += fabs(chassis_power_control->motor_speed_pid[i].out);
			}
			
//									
		}
		

}

