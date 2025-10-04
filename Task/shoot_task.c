/**
 * @file shoot_task.c
 * @author 何清华
 * @brief 包含对摩擦轮以及拨盘的控制
 * @version 0.1
 * @date 2022-03-23
 *
 * @copyright Copyright (c) 2022
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
#include "shoot_task.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"

#include "bsp_laser.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"

#define shoot_laser_on() laser_on()   //激光开启宏定义
#define shoot_laser_off() laser_off() //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

int TRY;   //发弹模式标志位  TRY为1：是单发，为0：连发
int X_flag = 0;
int X_flag_last = 0; 
int servo_flag = 0;
/***  速度环  ***/
//3508电机拨弹盘的pid
#define TRIGGER_KPX 4.5f     //6.0f//20.0f    //120.0f
#define TRIGGER_KIX 0.0f
#define TRIGGER_KDX 1.0f
#define TRIGGER_MAX_OUTX 20000.0f
#define TRIGGER_MAX_IOUTX 6000.0f

//pid_type_def avoid_shoot_speed={2.5f,0.002f,5.5f,10000.0f,20000.0f};
//pid_type_def avoid_shoot_angle={2.5f,0.002f,5.5f,10000.0f,20000.0f};

/* 角度环  */
pid_type_def bulletone={55.5f,0.15f,10.0f,3500.0f,1000.0f};

float debug_shoot[2];

int shoot_time_w;
/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
static void shoot_init(void);

/**
 * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void);

/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void);

/**
 * @brief          射击循环
 * @param[in]      void
 * @retval         返回can控制值
 */
static void shoot_control_loop(void);

/**
 * @brief          堵转倒转处理
 * @param[in]      void
 * @retval         void
 */
static void trigger_motor_turn_back(void);

/**
 * @brief          射击控制，控制拨弹电机角度，完成一次发射
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void);
static void servo_init();

float debug_shoot_angle1[2];
float debug_shoot_angle_set1[2];

float angle_wk[2];

extern uint16_t draw_init_flag;
extern uint8_t permiss_shoot_yaw ;
extern uint8_t permiss_shoot_pitch ;

//射击数据
shoot_control_t shoot_control;
//发送的电机电流

int bitstatus;
//extern int st_flag;

//static uint8_t move_flag =1;
static int flag1=0;

static int shoot_block;
float wkwk[2];
static uint8_t move_flag = 0;
int flagx=0;
static int16_t left_friction_can_set_current = 0, right_friction_can_set_current = 0, trigger_can_set_current = 0;

void fire_task(void const *pvParameters)
{
  //空闲一段时间
  vTaskDelay(SHOOT_TASK_INIT_TIME);

  //发射机构初始化
  shoot_init();
  servo_init();

  //判断底盘电机是否都在线
//  while (toe_is_error(LEFT_FRICTION_MOTOR_TOE) || toe_is_error(RIGHT_FRICTION_MOTOR_TOE))
//  {
//    vTaskDelay(SHOOT_CONTROL_TIME);
//  }

  while (1)
  {
    //射击模式选择
    shoot_set_mode();
    //发射机构数据更新
    shoot_feedback_update();
    //射击控制量计算
    shoot_control_loop();

    left_friction_can_set_current = shoot_control.left_fricition_motor.given_current;
    right_friction_can_set_current = shoot_control.right_fricition_motor.given_current;
    trigger_can_set_current = shoot_control.trigger_motor.given_current;

    bitstatus=!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);   // 获取电平
    
    /*********************卡弹标志位*********************/
    
    if(shoot_control.rc_s_time>=1000)
    {
		shoot_block=1;
	}
    else
    {
        shoot_block=0;
    }
    
    wkwk[0]=-shoot_control.left_fricition_motor.speed;
    wkwk[1]=shoot_control.right_fricition_motor.speed;
        
    if (!(toe_is_error(FRONT_LEFT_FRICTION_MOTOR_TOE) || toe_is_error(FRONT_RIGHT_FRICTION_MOTOR_TOE)))
    {
        if (toe_is_error(DBUS_TOE))
        {
            CAN_cmd_friction(0, 0);
            CAN_cmd_shoot(0);
            shoot_control.trigger_motor.set_angle = shoot_control.trigger_motor.angle;
        }
        else
        {
            CAN_cmd_friction(left_friction_can_set_current, right_friction_can_set_current);
            CAN_cmd_shoot(trigger_can_set_current);
        }
    }
    else
    {
        CAN_cmd_shoot(trigger_can_set_current);
        shoot_control.trigger_motor.set_angle=shoot_control.trigger_motor.angle;
    }
    
    if(toe_is_error(DBUS_TOE)) 
    {
        CAN_cmd_friction(0, 0);
        CAN_cmd_shoot(0);
        shoot_control.trigger_motor.set_angle = shoot_control.trigger_motor.angle;
    }
    debug_shoot_angle1[0]=shoot_control.left_fricition_motor.speed;
    debug_shoot_angle1[1]=-shoot_control.right_fricition_motor.speed;
    
    debug_shoot_angle_set1[0]=shoot_control.left_fricition_motor.speed_set;
    debug_shoot_angle_set1[1]=shoot_control.right_fricition_motor.speed_set;
    
    debug_shoot[0]=shoot_control.trigger_motor.set_angle;
    debug_shoot[1]=shoot_control.trigger_motor.angle;

    vTaskDelay(SHOOT_CONTROL_TIME);
  }
}

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */





void shoot_init(void)
{
  TRY=0;//1
  shoot_control.shoot_mode = SHOOT_READY;//SHOOT_STOP;
  //电机指针
  shoot_control.trigger_motor.motor_measure = get_trigger_motor_measure_point();
  shoot_control.left_fricition_motor.motor_measure = get_left_friction_motor_measure_point();
  shoot_control.right_fricition_motor.motor_measure = get_right_friction_motor_measure_point();
  //初始化PID
//    if(TRY)//TRY为1：选择3508电机pid
//    {
//        PID_init(&shoot_control.trigger_motor.motor_speed_pid, TRIGGER_KPX, TRIGGER_KIX, TRIGGER_KDX, TRIGGER_MAX_OUTX, TRIGGER_MAX_IOUTX); //拨弹电机
//    }
//    else//TRY为0：选择2006电机pid
//    {
        PID_init(&shoot_control.trigger_motor.motor_speed_pid, TRIGGER_KP, TRIGGER_KI, TRIGGER_KD, TRIGGER_MAX_OUT, TRIGGER_MAX_IOUT); //拨弹电机
//    }
//PID_init(&shoot_control.trigger_motor.motor_speed_pid, TRIGGER_KP, TRIGGER_KI, TRIGGER_KD, TRIGGER_MAX_OUT, TRIGGER_MAX_IOUT); //拨弹电机

  PID_init(&shoot_control.left_fricition_motor.motor_speed_pid, FRICTION_KP, FRICTION_KI, FRICTION_KD, FRICTION_MAX_OUT, FRICTION_MAX_IOUT); //左摩擦轮

  PID_init(&shoot_control.right_fricition_motor.motor_speed_pid, FRICTION_KP, FRICTION_KI, FRICTION_KD, FRICTION_MAX_OUT, FRICTION_MAX_IOUT); //右摩擦轮
  //更新数据
  shoot_feedback_update();
  //拨弹电机
  shoot_control.trigger_motor.ecd_count = 0;
  shoot_control.trigger_motor.angle = shoot_control.trigger_motor.motor_measure->angle;
  shoot_control.trigger_motor.given_current = 0;
  shoot_control.trigger_motor.set_angle = shoot_control.trigger_motor.angle;
  shoot_control.trigger_motor.speed = 0.0f;
  shoot_control.trigger_motor.speed_set = 0.0f;
  //左摩擦轮
  shoot_control.left_fricition_motor.accel = 0.0f;
  shoot_control.left_fricition_motor.speed = 0.0f;
  shoot_control.left_fricition_motor.speed_set = 0.0f;
  shoot_control.left_fricition_motor.given_current = 0;
  //右摩擦轮
  shoot_control.right_fricition_motor.accel = 0.0f;
  shoot_control.right_fricition_motor.speed = 0.0f;
  shoot_control.right_fricition_motor.speed_set = 0.0f;
  shoot_control.right_fricition_motor.given_current = 0;
	
  // M2006
//	shoot_control.avoid_shoot_motor.accel = 0.0f;
//  shoot_control.avoid_shoot_motor.speed = 0.0f;
//  shoot_control.avoid_shoot_motor.speed_set = 0.0f;
//  shoot_control.avoid_shoot_motor.given_current = 0;
}

/**
 * @brief          射击状态机设置，遥控器上拨1次发射数颗，一直处在上，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */

//int up1=0;
int medium1=0;
int down1=0;

int num=0;

int flagl=0;
int flagxl=0;
int time_flag=0;
static int kadan_flag;


float t;

int heat_flag;   //预超热量


int t1;

int shoot_acomplish;    //拨盘是否转到目标角度

int HEAT;

int Heat[2];

int block_time = 0;





//测试拨弹2006改写2024/3/21
static void shoot_set_mode(void)
 {
    //调试热量
    HEAT=shoot_control.heat;
    
    //Heat[0]=shoot_control.heat;//debug看2024.3.19
    //	HAL_Delay(100);
    Heat[1]=shoot_control.heat;

    //UI初始化
    if(rc_ctrl.key & KEY_PRESSED_OFFSET_B)
    {
        draw_init_flag=1;
    }

    /*   拨盘是否转到目标角度   */
    if(fabs(shoot_control.trigger_motor.set_angle-shoot_control.trigger_motor.angle)<=8.0f)  //8.0f
    {
		shoot_acomplish=1;
	}
    else
    {
		shoot_acomplish=0;
	}

    /***********获取热量，达到上限停止发射2024.3.19***********/
    //get_shoot_heat0_limit_and_heat0_42(&shoot_control.heat_limit, &shoot_control.heat);
    get_shoot_heat0_limit_and_heat0_17(&shoot_control.heat_limit, &shoot_control.heat);

    //热量限制2024.3.19
    if ((!toe_is_error(REFEREE_TOE)) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
    {
        if (shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET||shoot_control.shoot_mode == SHOOT_READY)
        {
            shoot_control.shoot_mode = SHOOT_READY;
            heat_flag=0;
        }
    }
    else
    {
        heat_flag=1;
    }

    //处于上档，进入发射模式
		//if (switch_is_up(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))//没加超热量限制
		if (heat_flag&&switch_is_up(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
			shoot_control.shoot_mode = SHOOT_BULLET;
			
			
       if(TRY)
        {
            if(flag1==0)
            {
                //Heat[0]=shoot_control.heat;//记录发射前的热量
                shoot_bullet_control();
                flag1=1;
            }
        
				}
    }
  //处于中档，开启摩擦轮
    else if ((rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT||switch_is_mid(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL])))
  //else if (switch_is_mid(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        shoot_control.shoot_mode = SHOOT_READY;
        flag1=0;

        if (!((shoot_control.press_l)||rc_ctrl.key & KEY_PRESSED_OFFSET_C)) 
            flagxl=1;
				if (((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))&&heat_flag)//&&heat_flag
//		if(heat_flag&&shoot_control.shoot_mode == SHOOT_BULLET && (shoot_control.press_l_time == PRESS_LONG_TIME || shoot_control.rc_s_time == RC_S_LONG_TIME))
				{
						shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;//连发
						move_flag = 0;
				}
        if (((shoot_control.press_l)||rc_ctrl.key & KEY_PRESSED_OFFSET_C)&&flagxl&&heat_flag&&shoot_acomplish)
        {
            //Heat[0]=shoot_control.heat;//记录发射前的热量
            
            flagxl=0;
            shoot_control.shoot_mode = SHOOT_BULLET;
            shoot_bullet_control();
            time_flag=1;
        }
        else
        {
            shoot_control.shoot_mode = SHOOT_READY;
//            if(rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)     //关闭摩擦轮
//            {
//                 shoot_control.shoot_mode = SHOOT_STOP;
//                 medium1=0;
//                 down1=1;
//            }
        }

    }
  //处于下档，关闭摩擦轮
		else if (switch_is_down(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
  //else if (switch_is_down(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
				if(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT||medium1==1)
        {
          shoot_control.shoot_mode = SHOOT_READY;
            if (!((shoot_control.press_l)||rc_ctrl.key & KEY_PRESSED_OFFSET_C)) flagxl=1;   // 
            //if (((shoot_control.press_l)||rc_ctrl.key & KEY_PRESSED_OFFSET_C)&&flagxl&&heat_flag)			
            if (((shoot_control.press_l)||rc_ctrl.key & KEY_PRESSED_OFFSET_C)&&flagxl&&heat_flag&&shoot_acomplish)    //
            {
                //Heat[0]=shoot_control.heat;//记录发射前的热量
                
                flagxl=0;
                shoot_control.shoot_mode = SHOOT_BULLET;
            shoot_bullet_control();
                time_flag=1;
            }
						 if (((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))&&heat_flag)//&&heat_flag
//		if(heat_flag&&shoot_control.shoot_mode == SHOOT_BULLET && (shoot_control.press_l_time == PRESS_LONG_TIME || shoot_control.rc_s_time == RC_S_LONG_TIME))
							{
									shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;//连发
									move_flag = 0;
							}
            else
            {
                shoot_control.shoot_mode = SHOOT_READY;
//                if(rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)     //关闭摩擦轮
//                {
//                     shoot_control.shoot_mode = SHOOT_STOP;
//                     medium1=0;
//                     down1=1;
//                }
            }
        }
        if (servo_flag == 0)
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1440); 
            servo_flag = 1;
        }
        
        if ( rc_ctrl.key & KEY_PRESSED_OFFSET_E&&servo_flag ==1)//X_flag - X_flag_last == 1//(!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) &&
        {
                laser_on();
                __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1440); 
        }
        if ( rc_ctrl.key & KEY_PRESSED_OFFSET_Q&&servo_flag ==1)
        {
                laser_off();
                __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 500); 
        }
    }
    else
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }

    angle_wk[1]=shoot_control.trigger_motor.set_angle;//上电角度
		
		

    /*       键盘控制       */


	if(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT||medium1==1)    //开启拨盘
    {
        shoot_control.shoot_mode = SHOOT_READY;
        medium1=1;
        down1=0;
        //如果处于中档并按下鼠标左键，进入发射模式
			if (shoot_control.press_l&&heat_flag)
        //if (medium1=1&&KEY_PRESSED_OFFSET_CTRL)
        {
           shoot_control.shoot_mode = SHOOT_BULLET;//单发
					 
					if(TRY)
					{
							if(flag1==0)
							{
									//Heat[0]=shoot_control.heat;//记录发射前的热量
									shoot_bullet_control();
									flag1=1;
							}
        
					}
				
        }
				
				
				
		//鼠标长按一直进入射击状态 保持连发
		//超热量限制
			 if (((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))&&heat_flag)//&&heat_flag
		//		if(heat_flag&&shoot_control.shoot_mode == SHOOT_BULLET && (shoot_control.press_l_time == PRESS_LONG_TIME || shoot_control.rc_s_time == RC_S_LONG_TIME))
				{
						shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;//连发
						move_flag = 0;
				}
				
	}
//	if(rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)     //关闭摩擦轮//CTRL
//	{
//		 shoot_control.shoot_mode = SHOOT_STOP;
//		 medium1=0;
//		 down1=1;
//	}

    if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z) {medium1=0;down1=0;}    //键盘数据清空


		

   /*       角度初始化          */
    if(1)
    {
        if(rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)
        {
            if(rc_ctrl.key & KEY_PRESSED_OFFSET_V)
            {
            shoot_control.trigger_motor.set_angle=0.955348969f;//0.955348969f;
            }
        }
    }
   
    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
    //	Heat[1]=shoot_control.heat;
    X_flag_last = X_flag;
    X_flag = ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_X);
    if (X_flag - X_flag_last == 1)
    {
        if (servo_flag == 0)
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 4700); //?180 2000
            servo_flag = 1;
        }
        else
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 800); //?0
            servo_flag = 0;
        }
    }
}




/**
 * @brief 射击数据更新
 *@param[in] void
 *@retval void
 */
static int wk1;
static int wk11;
static void shoot_feedback_update(void)
{
  shoot_control.trigger_motor.speed = shoot_control.trigger_motor.motor_measure->speed_rpm;
  //左摩擦轮
  shoot_control.left_fricition_motor.speed = shoot_control.left_fricition_motor.motor_measure->speed_rpm;
  //右摩擦轮
  shoot_control.right_fricition_motor.speed = shoot_control.right_fricition_motor.motor_measure->speed_rpm;

  //鼠标按键
  shoot_control.last_press_l = shoot_control.press_l;
  shoot_control.last_press_r = shoot_control.press_r;
  shoot_control.press_l = rc_ctrl.mouse.press_l;
  shoot_control.press_r = rc_ctrl.mouse.press_r;
  //长按计时
  if (shoot_control.press_l)
  {
    if (shoot_control.press_l_time < PRESS_LONG_TIME)
    {
      shoot_control.press_l_time++;
    }
  }
  else
  {
    shoot_control.press_l_time = 0;
  }

  if (shoot_control.press_r)
  {
    if (shoot_control.press_r_time < PRESS_LONG_TIME)
    {
      shoot_control.press_r_time++;
    }
  }
  else
  {
    shoot_control.press_r_time = 0;
  }
	

  /***********射击开关时间计时***********/
    if (shoot_control.shoot_mode == SHOOT_BULLET )
    {
        wk1=1;
    }

    if(wk1==1) shoot_control.rc_s_time++;

    if(shoot_acomplish==1)
    {
		wk1=0;
		shoot_control.rc_s_time=0;
	}

//	
//  if (shoot_control.shoot_mode == SHOOT_BULLET )
//  {
//    if (shoot_control.rc_s_time < RC_S_LONG_TIME)
//    {
//      shoot_control.rc_s_time++;
//    }
//  }
//  else
//  {
//    shoot_control.rc_s_time = 0;
//  }
}

/**
 * @brief          射击循环
 * @param[in]      void
 * @retval         返回can控制值
 */
int flag_l;
int flag_ll=0;
int flag_lll=0;

int mo_v=FRICTION_SPEED_SET;    //摩擦轮速度

static void shoot_control_loop(void)
{
  if (shoot_control.shoot_mode == SHOOT_STOP)
  {
    //激光关闭
    //shoot_laser_off();
    shoot_laser_on();
    //设置拨弹轮的速度
    shoot_control.trigger_motor.speed_set = 0.0f;
    shoot_control.left_fricition_motor.speed_set = 0.0f;
    shoot_control.right_fricition_motor.speed_set = 0.0f;

    //设置加速减速电流限制
    shoot_control.left_fricition_motor.motor_speed_pid.max_out = FRICTION_ACCEL_MAX_OUT;
    shoot_control.right_fricition_motor.motor_speed_pid.max_out = FRICTION_ACCEL_MAX_OUT;
  }
  else if (shoot_control.shoot_mode == SHOOT_READY)
  {
    //设置拨弹轮的速度
    shoot_control.trigger_motor.speed_set = 0.0f;
    //摩擦轮缓启动
    if (shoot_control.right_fricition_motor.speed > (FRICTION_SPEED_SET * 0.0f))
    {
      shoot_control.left_fricition_motor.motor_speed_pid.max_out = FRICTION_MAX_OUT;
      shoot_control.right_fricition_motor.motor_speed_pid.max_out = FRICTION_MAX_OUT;
    }
    

    //摩擦轮转速设置
    
    

   if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_X))
     {flag_ll=1;}
     if(rc_ctrl.key & KEY_PRESSED_OFFSET_X&&flag_ll)
     {
         mo_v+=10;
         flag_ll=0;
     }
 
        if(!(rc_ctrl.key & KEY_PRESSED_OFFSET_Z))
     {flag_lll=1;}
     if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z&&flag_lll)
     {
         mo_v-=10;
         flag_lll=0;
     }
		
//		if(!rc_ctrl.key & KEY_PRESSED_OFFSET_Z)
//		{flag_ll=1;}
//		if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z&&flag_ll)
//		{
//		  
//			mo_v=FRICTION_SPEED_SET;
//			flag_ll=0;
//	   	
//		}
//    shoot_control.left_fricition_motor.speed_set = -FRICTION_SPEED_SET;
//    shoot_control.right_fricition_motor.speed_set = FRICTION_SPEED_SET;
	  shoot_control.left_fricition_motor.speed_set = -mo_v-900;
    shoot_control.right_fricition_motor.speed_set = mo_v;
  }
  else if (shoot_control.shoot_mode == SHOOT_BULLET)
  {
      if(TRY)//TRY为1：是单发
		{
				//单发控制
				shoot_bullet_control();
			//  反拨
			  trigger_motor_turn_back();
		}
		else//TRY为0：是连发发
		{
				 shoot_control.trigger_motor.speed_set = CONTINUE_TRIGGER_SPEED;
		
		}
		
//		//单发控制
//    //shoot_bullet_control();
//    //设置拨弹轮的拨动速度,并开启堵转反转处理
//    shoot_control.trigger_motor.speed_set = CONTINUE_TRIGGER_SPEED;
//    // trigger_motor_turn_back();
  }
  else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
  {
    //设置拨弹轮的拨动速度,并开启堵转反转处理
    shoot_control.trigger_motor.speed_set = CONTINUE_TRIGGER_SPEED;
    // trigger_motor_turn_back();
  }

  if (shoot_control.shoot_mode != SHOOT_STOP)
  {
    shoot_laser_on(); //激光开启
  }

  //计算拨弹轮电机PID
	
	shoot_control.trigger_motor.angle=shoot_control.trigger_motor.motor_measure->angle;
	
	if(TRY)
	{
		PID_calc(&bulletone, shoot_control.trigger_motor.motor_measure->angle, shoot_control.trigger_motor.set_angle);
		PID_calc(&shoot_control.trigger_motor.motor_speed_pid, shoot_control.trigger_motor.speed, bulletone.out);
	}
	else
	{
		PID_calc(&shoot_control.trigger_motor.motor_speed_pid, shoot_control.trigger_motor.speed,shoot_control.trigger_motor.speed_set);
	}

  shoot_control.trigger_motor.given_current = (int16_t)(shoot_control.trigger_motor.motor_speed_pid.out);
	
	

  //计算摩擦轮电机PID
  PID_calc(&shoot_control.left_fricition_motor.motor_speed_pid, shoot_control.left_fricition_motor.speed, shoot_control.left_fricition_motor.speed_set);
  shoot_control.left_fricition_motor.given_current = (int16_t)(shoot_control.left_fricition_motor.motor_speed_pid.out);

  PID_calc(&shoot_control.right_fricition_motor.motor_speed_pid, shoot_control.right_fricition_motor.speed, shoot_control.right_fricition_motor.speed_set);
  shoot_control.right_fricition_motor.given_current = (int16_t)(shoot_control.right_fricition_motor.motor_speed_pid.out);
	
	
}

/**
 * @brief 卡弹检测及反转处理
 *
 */
static void trigger_motor_turn_back(void)
{
  if (shoot_control.block_time < BLOCK_TIME)
  {
    shoot_control.trigger_motor.speed_set = shoot_control.trigger_motor.speed_set;
  }
  else
  {
    shoot_control.trigger_motor.speed_set = -shoot_control.trigger_motor.speed_set;
  }
 
  if (fabs(shoot_control.trigger_motor.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
  {
    shoot_control.block_time++;
    shoot_control.reverse_time = 0;
  }
  else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
  {
    shoot_control.reverse_time++;
  }
  else
  {
    shoot_control.block_time = 0;
  }
}

/**
 * @brief          射击控制，控制拨弹电机角度，完成一次发射
 * @param[in]      void
 * @retval         void
 */


static void shoot_bullet_control(void)//2024.3.20步兵连发
{
	
	//初始角度   0.955348969
	
	//每次拨动一定角度
 
  {
    shoot_control.trigger_motor.set_angle -= shoot_control.trigger_motor.angle - 1300.0f;//60.5f;//theta_format(shoot_control.trigger_motor.angle/18 + 60.0f);

  }
	
	
  //到达角度判断
//  if (theta_format(shoot_control.trigger_motor.set_angle - shoot_control.trigger_motor.angle) > 0)
//  {
//    //没到达一直设置旋转速度
//    shoot_control.trigger_motor.speed_set = TRIGGER_SPEED;
//  }
//  else
//  {
//    ;
//  }
	//angle_wk[1]=shoot_control.trigger_motor.set_angle;
	
	angle_wk[0]=angle_wk[1];   //记录上次角度

//	HAL_Delay(100);
//	Heat[1]=shoot_control.heat;//记录理论拨盘转完后的热量
	

//	if(fabs(shoot_control.trigger_motor.set_angle - shoot_control.trigger_motor.motor_measure->angle)<0.5f)
//	{
//		 ;
//	}
//	else
//	{
//	   shoot_control.trigger_motor.speed_set = TRIGGER_SPEED;
//	}
//	
}


void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	
  UNUSED(GPIO_Pin);
  //HAL_Delay(10); 
	shoot_control.shoot_mode = SHOOT_READY;

}

static void bullet_one()
{

}

static void servo_init()
{
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Base_Start(&htim8);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  //  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 700); // 660 1600
	
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,4700);
}