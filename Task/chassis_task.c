/**
 * @file chassis_task.c/h
 * @author ���廪��������
 * @brief ���̿��������߳�
 * @version 0.1
 * @date 2022-03-06
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
#include "chassis_task.h"
#include "chassis_power_control.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "main.h"
#include "arm_math.h"
#include "detect_task.h"
#include "INS_task.h"
#include "custom_ui_draw.h"
#include "CAN_receive.h"
#include "referee.h"

extern tank_ui_t tank;

//extern super_capacity_t super_capacity;
extern ext_game_robot_state_t robot_state;

uint16_t CHASSIS_MOTOR_SPEED_MAX_OUT=2000; 
float NORMAL_MAX_CHASSIS_SPEED_X=1.0f;
/**
*	@brief  ��ͬ�ȼ����ʿ���
*	@retval  none
*/
static void chassis_level_power(void);

/**
 * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
 * @retval         none
 */
static void chassis_init(void);

/**
 * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
 * @retval         none
 */
static void chassis_set_mode(void);

/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @param[out]     chassis_move_update:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_feedback_update(void);

/**
 * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
 * @retval         none
 */
static void chassis_set_contorl(void);

/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_control_loop(void);

static void chassis_power_control_update(void);//���³��繦��
void chassis_data_send(void);


//�����˶�����
chassis_move_t chassis_move;

/**
 * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: ��
 * @retval         none
 */
 extern int up,medium,down;
 extern int medium1,down1;
 extern int up2,medium2,down2;
 
 extern super_capacity_t super_capacity;

void chassis_task(void const *pvParameters)
{
  //����һ��ʱ��
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
  //���̳�ʼ��
  chassis_init();
  //�жϵ��̵���Ƿ�����
	

  while (1)
  {
    //���õ��̿���ģʽ
    chassis_set_mode();
    //�������ݸ���
    chassis_feedback_update();
    //���̿���������
    chassis_set_contorl();
    //�ȼ��������޸�ֵ
    chassis_level_power();
    chassis_move.vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X * chassis_power_scale;
    chassis_move.vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X * chassis_power_scale;

    //���̿���PID����
    chassis_control_loop();
    //����
    //chassis_data_send();

    cal_draw_gimbal_relative_angle_tangle(&tank);

    //cal_capacity(&super_capacity);�������ݣ�
    chassis_power_control_update();
    //ȷ������һ��������ߣ� ����CAN���ư����Ա����յ�
    if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
    {
      //��ң�������ߵ�ʱ�򣬷��͸����̵�������.
      if (toe_is_error(DBUS_TOE))
      {
        CAN_cmd_chassis(0, 0, 0, 0);

            
            {
                up=0;medium=0;down=0;
              medium1=0;down1=0;
              up2=0;medium2=0;down2=0;				//���������Զ����
            }
            
      }
      else
      {
        //���Ϳ��Ƶ���
        CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                        chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
        //CAN_cmd_chassis(0, 0, 0, 0);
      }
    }
		
		 

    //ϵͳ��ʱ
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
  }
}

/**
 * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
 * @retval         none
 */
static void chassis_init(void)
{
  const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
  uint8_t i;

  //��ȡ��̨�������ָ��
  chassis_move.chassis_yaw_motor = get_yaw_motor_point();
  chassis_move.chassis_pitch_motor = get_pitch_motor_point();
  //��ȡ���̵������ָ�룬��ʼ��PID
  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
    PID_init(&chassis_move.motor_speed_pid[i], CHASSIS_MOTOR_SPEED_KP, CHASSIS_MOTOR_SPEED_KI, CHASSIS_MOTOR_SPEED_KD, CHASSIS_MOTOR_SPEED_MAX_OUT, CHASSIS_MOTOR_SPEED_MAX_IOUT);
  }
  //��ʼ���Ƕ�PID(��ת������̨)
  PID_init(&chassis_move.chassis_angle_pid, CHASSIS_ANGLE_KP, CHASSIS_ANGLE_KI, CHASSIS_ANGLE_KD, CHASSIS_ANGLE_MAX_OUT, CHASSIS_ANGLE_MAX_IOUT);

  //��һ���˲�����б����������
  first_order_filter_init(&chassis_move.chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
  first_order_filter_init(&chassis_move.chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);


  //��� ��С�ٶ�
  chassis_move.vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
  chassis_move.vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

  chassis_move.vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
  chassis_move.vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

  chassis_move.top_max_speed = TOP_MAX_CHASSIS_SPEED;
  chassis_move.top_min_speed = -TOP_MAX_CHASSIS_SPEED;
	


  //����һ������
  chassis_feedback_update();
}

/**
 * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
 * @retval         none
 */
static void chassis_set_mode()
{
  chassis_behaviour_mode_set(&chassis_move);
}

/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @retval         none
 */
static void chassis_feedback_update(void)
{
  uint8_t i = 0;
  for (i = 0; i < 4; i++)
  {
    //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
    chassis_move.motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move.motor_chassis[i].chassis_motor_measure->speed_rpm;
    chassis_move.motor_chassis[i].accel = chassis_move.motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
  }

  //���µ��������ٶ� x��ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
  chassis_move.vx = (chassis_move.motor_chassis[0].speed - chassis_move.motor_chassis[1].speed - chassis_move.motor_chassis[2].speed + chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
  chassis_move.vy = (chassis_move.motor_chassis[0].speed + chassis_move.motor_chassis[1].speed - chassis_move.motor_chassis[2].speed - chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
  chassis_move.wz = (-chassis_move.motor_chassis[0].speed - chassis_move.motor_chassis[1].speed - chassis_move.motor_chassis[2].speed - chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

}

/**
 * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
 * @retval         none
 */
static void chassis_set_contorl(void)
{
  chassis_behaviour_control_set(&chassis_move);
}
////////////////���ID/////////////////
/////////1********ǰ*******2///////////
/////////*                 *///////////
/////////*        x        *///////////
/////////��       ����y      ��//////////
/////////*                 *///////////
/////////*                 *///////////
/////////4********��********3//////////
///////////////////////////////////////
/**
 * @brief          �ĸ������ٶ���ͨ�������������������
 * @param[in]      vx_set: �����ٶ�
 * @param[in]      vy_set: �����ٶ�
 * @param[in]      wz_set: ��ת�ٶ�
 * @param[out]     wheel_speed: �ĸ������ٶ�
 * @retval         none
 */
static void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4])
{
  wheel_speed[0] = vx_set - vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[1] = -vx_set - vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[2] = -vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[3] = vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;

}

/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_control_loop(void)
{
  float max_vector = 0.0f, vector_rate = 0.0f;
  float temp = 0.0f;
  float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t i = 0;
  
   if(chassis_move.vx_set<0.1||chassis_move.vx_set>-0.1)
  {
      if(chassis_move.vy_set>0.5||chassis_move.vy_set<-0.5)
      {
        switch(robot_state.robot_level)
        {
            case 1:{CHASSIS_MOTOR_SPEED_MAX_OUT=4000;}break;//3000
            case 2:{CHASSIS_MOTOR_SPEED_MAX_OUT=3200;}break;
            case 3:{CHASSIS_MOTOR_SPEED_MAX_OUT=3400;}break;
            case 4:{CHASSIS_MOTOR_SPEED_MAX_OUT=3500;}break;
            case 5:{CHASSIS_MOTOR_SPEED_MAX_OUT=3600;}break;
            case 6:{CHASSIS_MOTOR_SPEED_MAX_OUT=3700;}break;//
            case 7:{CHASSIS_MOTOR_SPEED_MAX_OUT=3800;}break;//
            case 8:{CHASSIS_MOTOR_SPEED_MAX_OUT=3900;}break;//
            case 9:{CHASSIS_MOTOR_SPEED_MAX_OUT=3950;}break;//
            case 10:{CHASSIS_MOTOR_SPEED_MAX_OUT=4000;}break;//
            default:break;
          }
      }
  }

  //�����˶��ֽ�
  chassis_vector_to_mecanum_wheel_speed(chassis_move.vx_set, chassis_move.vy_set, chassis_move.wz_set, wheel_speed);
  
  if (chassis_move.chassis_mode == CHASSIS_OPEN || chassis_move.chassis_mode == CHASSIS_ZERO_FORCE)
  {

    for (i = 0; i < 4; i++)
    {
      chassis_move.motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
    }
    // raw����ֱ�ӷ���
    return;
  }

  //�������ӿ�������ٶȣ�������������ٶ�
  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis[i].speed_set = wheel_speed[i];
    temp = fabs(chassis_move.motor_chassis[i].speed_set);
    if (max_vector < temp)
    {
      max_vector = temp;
    }
  }

  if (max_vector > MAX_WHEEL_SPEED)
  {
    vector_rate = MAX_WHEEL_SPEED / max_vector;
    for (i = 0; i < 4; i++)
    {
      chassis_move.motor_chassis[i].speed_set *= vector_rate;
    }
  }

  //����pid
  for (i = 0; i < 4; i++)
  {
    PID_calc_chassis(&chassis_move.motor_speed_pid[i], chassis_move.motor_chassis[i].speed, chassis_move.motor_chassis[i].speed_set);
  }
//	robot_state.robot_level = 1;
  chassis_power_control(&chassis_move);


  //��ֵ����ֵ
  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis[i].give_current = (int16_t)(chassis_move.motor_speed_pid[i].out);
  }
}
/**
 * @brief          ������ݷŵ����
 * @param[in]      voltage: ���ݵ�ѹ
 * @param[in]      power: �ŵ繦��
 * @retval         �ŵ����
 */


static void chassis_power_control_update(void)
{
    // ����δ�������ֵ��ѹ�����ֵ��̾�ֹ
    
        if ( PowerData[1] >= CAP_CHARGE_VOLTAGE_THRESHOLD) {
//            super_cap_send_power(PowerData[4]); // ���ó�ʼ��������

			float power = 0;
            switch (robot_state.robot_level) {
                case 0: power = 60.0f; break;//20s
			    case 1: power = 45.0f; break;
                case 2: power = 45.0f; break;//12s
                case 3: power = 45.0f; break;
                case 4: power = 45.0f; break;
                case 5: power = 45.0f; break;
                case 6: power = 45.0f; break;
                case 7: power = 45.0f; break;
                case 8: power = 45.0f; break;
                case 9: power = 45.0f; break;
                case 10: power = 45.0f; break;
							
                default: break;
            }
            super_cap_send_power(power);
        } else {
            chassis_power_scale = 0.8f;
					  super_cap_send_power(0);
            return;
        }
    

    // ���ݵ���״̬���¹���ϵ��
    if ( PowerData[1] <= CAP_LOW_VOLTAGE_THRESHOLD || fabs( PowerData[2]) >= CAP_HIGH_CURRENT_THRESHOLD) 
			{
        // ���ݵ�ѹ���ͻ��������ʱ�����͹���
        chassis_power_scale = 0.8f;  // ˥��ϵ��,����
    } else {
        chassis_power_scale = 1.0f;
		}
}
		


static void chassis_level_power(void)
{
	switch(robot_state.robot_level)//�޳����
	{
		case 0  :{CHASSIS_MOTOR_SPEED_MAX_OUT=3000;NORMAL_MAX_CHASSIS_SPEED_X=2.0;}break;
		case 1:{CHASSIS_MOTOR_SPEED_MAX_OUT=3000;NORMAL_MAX_CHASSIS_SPEED_X=2.0;}break;//60//��̺�����42w
		case 2:{CHASSIS_MOTOR_SPEED_MAX_OUT=3000;NORMAL_MAX_CHASSIS_SPEED_X=2.4;}break;//65
		case 3:{CHASSIS_MOTOR_SPEED_MAX_OUT=3000;NORMAL_MAX_CHASSIS_SPEED_X=2.6;}break;//70
		case 4:{CHASSIS_MOTOR_SPEED_MAX_OUT=3000;NORMAL_MAX_CHASSIS_SPEED_X=2.8;}break;//75
		case 5:{CHASSIS_MOTOR_SPEED_MAX_OUT=3000;NORMAL_MAX_CHASSIS_SPEED_X=3.0;}break;//80
		case 6:{CHASSIS_MOTOR_SPEED_MAX_OUT=3000;NORMAL_MAX_CHASSIS_SPEED_X=3.2;}break;//85
		case 7:{CHASSIS_MOTOR_SPEED_MAX_OUT=3000;NORMAL_MAX_CHASSIS_SPEED_X=3.4;}break;//90
		case 8:{CHASSIS_MOTOR_SPEED_MAX_OUT=3000;NORMAL_MAX_CHASSIS_SPEED_X=3.6;}break;//95
		case 9:{CHASSIS_MOTOR_SPEED_MAX_OUT=3000;NORMAL_MAX_CHASSIS_SPEED_X=3.8;}break;//100
		case 10:{CHASSIS_MOTOR_SPEED_MAX_OUT=3000;NORMAL_MAX_CHASSIS_SPEED_X=2.4;}break;//100
		default:break;


	}
}