/**
 * @file shoot_task.c
 * @author ���廪
 * @brief ������Ħ�����Լ����̵Ŀ���
 * @version 0.1
 * @date 2022-03-23
 *
 * @copyright Copyright (c) 2022
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

#define shoot_laser_on() laser_on()   //���⿪���궨��
#define shoot_laser_off() laser_off() //����رպ궨��
//΢������IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

int TRY;   //����ģʽ��־λ  TRYΪ1���ǵ�����Ϊ0������
int X_flag = 0;
int X_flag_last = 0; 
int servo_flag = 0;
/***  �ٶȻ�  ***/
//3508��������̵�pid
#define TRIGGER_KPX 4.5f     //6.0f//20.0f    //120.0f
#define TRIGGER_KIX 0.0f
#define TRIGGER_KDX 1.0f
#define TRIGGER_MAX_OUTX 20000.0f
#define TRIGGER_MAX_IOUTX 6000.0f

//pid_type_def avoid_shoot_speed={2.5f,0.002f,5.5f,10000.0f,20000.0f};
//pid_type_def avoid_shoot_angle={2.5f,0.002f,5.5f,10000.0f,20000.0f};

/* �ǶȻ�  */
pid_type_def bulletone={55.5f,0.15f,10.0f,3500.0f,1000.0f};

float debug_shoot[2];

int shoot_time_w;
/**
 * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
 * @param[in]      void
 * @retval         ���ؿ�
 */
static void shoot_init(void);

/**
 * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void);

/**
 * @brief          ������ݸ���
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void);

/**
 * @brief          ���ѭ��
 * @param[in]      void
 * @retval         ����can����ֵ
 */
static void shoot_control_loop(void);

/**
 * @brief          ��ת��ת����
 * @param[in]      void
 * @retval         void
 */
static void trigger_motor_turn_back(void);

/**
 * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
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

//�������
shoot_control_t shoot_control;
//���͵ĵ������

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
  //����һ��ʱ��
  vTaskDelay(SHOOT_TASK_INIT_TIME);

  //���������ʼ��
  shoot_init();
  servo_init();

  //�жϵ��̵���Ƿ�����
//  while (toe_is_error(LEFT_FRICTION_MOTOR_TOE) || toe_is_error(RIGHT_FRICTION_MOTOR_TOE))
//  {
//    vTaskDelay(SHOOT_CONTROL_TIME);
//  }

  while (1)
  {
    //���ģʽѡ��
    shoot_set_mode();
    //����������ݸ���
    shoot_feedback_update();
    //�������������
    shoot_control_loop();

    left_friction_can_set_current = shoot_control.left_fricition_motor.given_current;
    right_friction_can_set_current = shoot_control.right_fricition_motor.given_current;
    trigger_can_set_current = shoot_control.trigger_motor.given_current;

    bitstatus=!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);   // ��ȡ��ƽ
    
    /*********************������־λ*********************/
    
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
 * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
 * @param[in]      void
 * @retval         ���ؿ�
 */





void shoot_init(void)
{
  TRY=0;//1
  shoot_control.shoot_mode = SHOOT_READY;//SHOOT_STOP;
  //���ָ��
  shoot_control.trigger_motor.motor_measure = get_trigger_motor_measure_point();
  shoot_control.left_fricition_motor.motor_measure = get_left_friction_motor_measure_point();
  shoot_control.right_fricition_motor.motor_measure = get_right_friction_motor_measure_point();
  //��ʼ��PID
//    if(TRY)//TRYΪ1��ѡ��3508���pid
//    {
//        PID_init(&shoot_control.trigger_motor.motor_speed_pid, TRIGGER_KPX, TRIGGER_KIX, TRIGGER_KDX, TRIGGER_MAX_OUTX, TRIGGER_MAX_IOUTX); //�������
//    }
//    else//TRYΪ0��ѡ��2006���pid
//    {
        PID_init(&shoot_control.trigger_motor.motor_speed_pid, TRIGGER_KP, TRIGGER_KI, TRIGGER_KD, TRIGGER_MAX_OUT, TRIGGER_MAX_IOUT); //�������
//    }
//PID_init(&shoot_control.trigger_motor.motor_speed_pid, TRIGGER_KP, TRIGGER_KI, TRIGGER_KD, TRIGGER_MAX_OUT, TRIGGER_MAX_IOUT); //�������

  PID_init(&shoot_control.left_fricition_motor.motor_speed_pid, FRICTION_KP, FRICTION_KI, FRICTION_KD, FRICTION_MAX_OUT, FRICTION_MAX_IOUT); //��Ħ����

  PID_init(&shoot_control.right_fricition_motor.motor_speed_pid, FRICTION_KP, FRICTION_KI, FRICTION_KD, FRICTION_MAX_OUT, FRICTION_MAX_IOUT); //��Ħ����
  //��������
  shoot_feedback_update();
  //�������
  shoot_control.trigger_motor.ecd_count = 0;
  shoot_control.trigger_motor.angle = shoot_control.trigger_motor.motor_measure->angle;
  shoot_control.trigger_motor.given_current = 0;
  shoot_control.trigger_motor.set_angle = shoot_control.trigger_motor.angle;
  shoot_control.trigger_motor.speed = 0.0f;
  shoot_control.trigger_motor.speed_set = 0.0f;
  //��Ħ����
  shoot_control.left_fricition_motor.accel = 0.0f;
  shoot_control.left_fricition_motor.speed = 0.0f;
  shoot_control.left_fricition_motor.speed_set = 0.0f;
  shoot_control.left_fricition_motor.given_current = 0;
  //��Ħ����
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
 * @brief          ���״̬�����ã�ң�����ϲ�1�η������ţ�һֱ�����ϣ���������䣬����3min׼��ʱ�������ӵ�
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

int heat_flag;   //Ԥ������


int t1;

int shoot_acomplish;    //�����Ƿ�ת��Ŀ��Ƕ�

int HEAT;

int Heat[2];

int block_time = 0;





//���Բ���2006��д2024/3/21
static void shoot_set_mode(void)
 {
    //��������
    HEAT=shoot_control.heat;
    
    //Heat[0]=shoot_control.heat;//debug��2024.3.19
    //	HAL_Delay(100);
    Heat[1]=shoot_control.heat;

    //UI��ʼ��
    if(rc_ctrl.key & KEY_PRESSED_OFFSET_B)
    {
        draw_init_flag=1;
    }

    /*   �����Ƿ�ת��Ŀ��Ƕ�   */
    if(fabs(shoot_control.trigger_motor.set_angle-shoot_control.trigger_motor.angle)<=8.0f)  //8.0f
    {
		shoot_acomplish=1;
	}
    else
    {
		shoot_acomplish=0;
	}

    /***********��ȡ�������ﵽ����ֹͣ����2024.3.19***********/
    //get_shoot_heat0_limit_and_heat0_42(&shoot_control.heat_limit, &shoot_control.heat);
    get_shoot_heat0_limit_and_heat0_17(&shoot_control.heat_limit, &shoot_control.heat);

    //��������2024.3.19
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

    //�����ϵ������뷢��ģʽ
		//if (switch_is_up(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))//û�ӳ���������
		if (heat_flag&&switch_is_up(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
			shoot_control.shoot_mode = SHOOT_BULLET;
			
			
       if(TRY)
        {
            if(flag1==0)
            {
                //Heat[0]=shoot_control.heat;//��¼����ǰ������
                shoot_bullet_control();
                flag1=1;
            }
        
				}
    }
  //�����е�������Ħ����
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
						shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;//����
						move_flag = 0;
				}
        if (((shoot_control.press_l)||rc_ctrl.key & KEY_PRESSED_OFFSET_C)&&flagxl&&heat_flag&&shoot_acomplish)
        {
            //Heat[0]=shoot_control.heat;//��¼����ǰ������
            
            flagxl=0;
            shoot_control.shoot_mode = SHOOT_BULLET;
            shoot_bullet_control();
            time_flag=1;
        }
        else
        {
            shoot_control.shoot_mode = SHOOT_READY;
//            if(rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)     //�ر�Ħ����
//            {
//                 shoot_control.shoot_mode = SHOOT_STOP;
//                 medium1=0;
//                 down1=1;
//            }
        }

    }
  //�����µ����ر�Ħ����
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
                //Heat[0]=shoot_control.heat;//��¼����ǰ������
                
                flagxl=0;
                shoot_control.shoot_mode = SHOOT_BULLET;
            shoot_bullet_control();
                time_flag=1;
            }
						 if (((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))&&heat_flag)//&&heat_flag
//		if(heat_flag&&shoot_control.shoot_mode == SHOOT_BULLET && (shoot_control.press_l_time == PRESS_LONG_TIME || shoot_control.rc_s_time == RC_S_LONG_TIME))
							{
									shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;//����
									move_flag = 0;
							}
            else
            {
                shoot_control.shoot_mode = SHOOT_READY;
//                if(rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)     //�ر�Ħ����
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

    angle_wk[1]=shoot_control.trigger_motor.set_angle;//�ϵ�Ƕ�
		
		

    /*       ���̿���       */


	if(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT||medium1==1)    //��������
    {
        shoot_control.shoot_mode = SHOOT_READY;
        medium1=1;
        down1=0;
        //��������е������������������뷢��ģʽ
			if (shoot_control.press_l&&heat_flag)
        //if (medium1=1&&KEY_PRESSED_OFFSET_CTRL)
        {
           shoot_control.shoot_mode = SHOOT_BULLET;//����
					 
					if(TRY)
					{
							if(flag1==0)
							{
									//Heat[0]=shoot_control.heat;//��¼����ǰ������
									shoot_bullet_control();
									flag1=1;
							}
        
					}
				
        }
				
				
				
		//��곤��һֱ�������״̬ ��������
		//����������
			 if (((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))&&heat_flag)//&&heat_flag
		//		if(heat_flag&&shoot_control.shoot_mode == SHOOT_BULLET && (shoot_control.press_l_time == PRESS_LONG_TIME || shoot_control.rc_s_time == RC_S_LONG_TIME))
				{
						shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;//����
						move_flag = 0;
				}
				
	}
//	if(rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)     //�ر�Ħ����//CTRL
//	{
//		 shoot_control.shoot_mode = SHOOT_STOP;
//		 medium1=0;
//		 down1=1;
//	}

    if(rc_ctrl.key & KEY_PRESSED_OFFSET_Z) {medium1=0;down1=0;}    //�����������


		

   /*       �Ƕȳ�ʼ��          */
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
   
    //�����̨״̬�� ����״̬���͹ر����
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
 * @brief ������ݸ���
 *@param[in] void
 *@retval void
 */
static int wk1;
static int wk11;
static void shoot_feedback_update(void)
{
  shoot_control.trigger_motor.speed = shoot_control.trigger_motor.motor_measure->speed_rpm;
  //��Ħ����
  shoot_control.left_fricition_motor.speed = shoot_control.left_fricition_motor.motor_measure->speed_rpm;
  //��Ħ����
  shoot_control.right_fricition_motor.speed = shoot_control.right_fricition_motor.motor_measure->speed_rpm;

  //��갴��
  shoot_control.last_press_l = shoot_control.press_l;
  shoot_control.last_press_r = shoot_control.press_r;
  shoot_control.press_l = rc_ctrl.mouse.press_l;
  shoot_control.press_r = rc_ctrl.mouse.press_r;
  //������ʱ
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
	

  /***********�������ʱ���ʱ***********/
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
 * @brief          ���ѭ��
 * @param[in]      void
 * @retval         ����can����ֵ
 */
int flag_l;
int flag_ll=0;
int flag_lll=0;

int mo_v=FRICTION_SPEED_SET;    //Ħ�����ٶ�

static void shoot_control_loop(void)
{
  if (shoot_control.shoot_mode == SHOOT_STOP)
  {
    //����ر�
    //shoot_laser_off();
    shoot_laser_on();
    //���ò����ֵ��ٶ�
    shoot_control.trigger_motor.speed_set = 0.0f;
    shoot_control.left_fricition_motor.speed_set = 0.0f;
    shoot_control.right_fricition_motor.speed_set = 0.0f;

    //���ü��ټ��ٵ�������
    shoot_control.left_fricition_motor.motor_speed_pid.max_out = FRICTION_ACCEL_MAX_OUT;
    shoot_control.right_fricition_motor.motor_speed_pid.max_out = FRICTION_ACCEL_MAX_OUT;
  }
  else if (shoot_control.shoot_mode == SHOOT_READY)
  {
    //���ò����ֵ��ٶ�
    shoot_control.trigger_motor.speed_set = 0.0f;
    //Ħ���ֻ�����
    if (shoot_control.right_fricition_motor.speed > (FRICTION_SPEED_SET * 0.0f))
    {
      shoot_control.left_fricition_motor.motor_speed_pid.max_out = FRICTION_MAX_OUT;
      shoot_control.right_fricition_motor.motor_speed_pid.max_out = FRICTION_MAX_OUT;
    }
    

    //Ħ����ת������
    
    

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
      if(TRY)//TRYΪ1���ǵ���
		{
				//��������
				shoot_bullet_control();
			//  ����
			  trigger_motor_turn_back();
		}
		else//TRYΪ0����������
		{
				 shoot_control.trigger_motor.speed_set = CONTINUE_TRIGGER_SPEED;
		
		}
		
//		//��������
//    //shoot_bullet_control();
//    //���ò����ֵĲ����ٶ�,��������ת��ת����
//    shoot_control.trigger_motor.speed_set = CONTINUE_TRIGGER_SPEED;
//    // trigger_motor_turn_back();
  }
  else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
  {
    //���ò����ֵĲ����ٶ�,��������ת��ת����
    shoot_control.trigger_motor.speed_set = CONTINUE_TRIGGER_SPEED;
    // trigger_motor_turn_back();
  }

  if (shoot_control.shoot_mode != SHOOT_STOP)
  {
    shoot_laser_on(); //���⿪��
  }

  //���㲦���ֵ��PID
	
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
	
	

  //����Ħ���ֵ��PID
  PID_calc(&shoot_control.left_fricition_motor.motor_speed_pid, shoot_control.left_fricition_motor.speed, shoot_control.left_fricition_motor.speed_set);
  shoot_control.left_fricition_motor.given_current = (int16_t)(shoot_control.left_fricition_motor.motor_speed_pid.out);

  PID_calc(&shoot_control.right_fricition_motor.motor_speed_pid, shoot_control.right_fricition_motor.speed, shoot_control.right_fricition_motor.speed_set);
  shoot_control.right_fricition_motor.given_current = (int16_t)(shoot_control.right_fricition_motor.motor_speed_pid.out);
	
	
}

/**
 * @brief ������⼰��ת����
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
 * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
 * @param[in]      void
 * @retval         void
 */


static void shoot_bullet_control(void)//2024.3.20��������
{
	
	//��ʼ�Ƕ�   0.955348969
	
	//ÿ�β���һ���Ƕ�
 
  {
    shoot_control.trigger_motor.set_angle -= shoot_control.trigger_motor.angle - 1300.0f;//60.5f;//theta_format(shoot_control.trigger_motor.angle/18 + 60.0f);

  }
	
	
  //����Ƕ��ж�
//  if (theta_format(shoot_control.trigger_motor.set_angle - shoot_control.trigger_motor.angle) > 0)
//  {
//    //û����һֱ������ת�ٶ�
//    shoot_control.trigger_motor.speed_set = TRIGGER_SPEED;
//  }
//  else
//  {
//    ;
//  }
	//angle_wk[1]=shoot_control.trigger_motor.set_angle;
	
	angle_wk[0]=angle_wk[1];   //��¼�ϴνǶ�

//	HAL_Delay(100);
//	Heat[1]=shoot_control.heat;//��¼���۲���ת��������
	

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