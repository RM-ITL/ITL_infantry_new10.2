/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2



//底盘PID参数定义
//电机速度PID
#define CHASSIS_MOTOR_SPEED_KP 9000//5000//9000//15000 10000
#define CHASSIS_MOTOR_SPEED_KI 0.0125
#define CHASSIS_MOTOR_SPEED_KD 0//0.1
extern uint16_t CHASSIS_MOTOR_SPEED_MAX_OUT; //九3200(1.2)//七2600（1.3）//二2100（1.1）//六2750(1.2)//三2250(1.0)//四2300（1.2）//五2500（1.2）//八3000(1.3)//十4000(1.5)//一2000//7000//14000  //15000
#define CHASSIS_MOTOR_SPEED_MAX_IOUT 1000//10000
//底盘跟随云台旋转PID
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

//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f

// m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 3.7f//2.0f//3.0f//4.0f//8.0f 4.0f
//底盘运动过程最大前进速度
extern float NORMAL_MAX_CHASSIS_SPEED_X; //2.0f//1.0f//2.0f//3.0//1.0f//2.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f //2.5  //1.5f

#define NORMAL_MAX_CHASSIS_SPEED_Z 6.0f//3.0f

//底盘小陀螺速度
#define CHASSIS_TOP_SPEED 9//8  // 6.0f//7
//底盘小陀螺时最大平移速度
#define TOP_MAX_CHASSIS_SPEED 4.0f

#define CAP_CHARGE_VOLTAGE_THRESHOLD 18.0f  // 电容充电电压阈值
#define CAP_LOW_VOLTAGE_THRESHOLD   18.0f   // 电容低压阈值
#define CAP_HIGH_CURRENT_THRESHOLD  8.0f    // 电容高电流阈值
//#define INITIAL_POWER_LIMIT        40.0f    // 初始功率限制 40W


// 添加功率控制相关变量
static float chassis_power_scale = 1.0f;    // 底盘功率限制系数
//static float cap_ready = 0;              // 电容就绪标志
static float discharge_current = 0.0f;       // 放电电流
typedef enum
{
  CHASSIS_ZERO_FORCE = 0,   //底盘无力, 跟没上电那样
  CHASSIS_NO_MOVE,          //底盘保持不动
  CHASSIS_FOLLOW_GIMBAL,    //底盘跟随云台旋转
  CHASSIS_TOP,              
  CHASSIS_NO_FOLLOW_GIMBAL, //底盘不跟随云台旋转
  CHASSIS_OPEN              //遥控器的值乘以比例成电流值 直接发送到can总线上
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  float accel;          //加速度
  float speed;          //当前速度
  float speed_set;      //设定速度
  int16_t give_current; //发送电流
} chassis_motor_t;

typedef struct
{
  const gimbal_motor_t *chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角.
  const gimbal_motor_t *chassis_pitch_motor; //底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
  const float *chassis_INS_angle;            //获取陀螺仪解算出的欧拉角指针
  chassis_mode_e chassis_mode;               //底盘控制状态机
  chassis_motor_t motor_chassis[4];          //底盘电机数据
  pid_type_def motor_speed_pid[4];           //底盘电机速度pid
  pid_type_def chassis_angle_pid;            //底盘跟随角度pid

  first_order_filter_type_t chassis_cmd_slow_set_vx; //使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy; //使用一阶低通滤波减缓设定值

  float vx;                         //底盘速度 前进方向 前为正，单位 m/s
  float vy;                         //底盘速度 左右方向 左为正  单位 m/s
  float wz;                         //底盘旋转角速度，逆时针为正 单位 rad/s
  float vx_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
  float vy_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
  float wz_set;                     //底盘设定旋转角速度，逆时针为正 单位 rad/s
  float chassis_relative_angle;     //底盘与云台的相对角度，单位 rad
  float chassis_relative_angle_set; //设置相对云台控制角度
  float chassis_yaw_set;

  float vx_max_speed;  //前进方向最大速度 单位m/s
  float vx_min_speed;  //后退方向最大速度 单位m/s
  float vy_max_speed;  //左方向最大速度 单位m/s
  float vy_min_speed;  //右方向最大速度 单位m/s
  float top_max_speed; //小陀螺平移正方向最大速度
  float top_min_speed; //小陀螺平移反方向最大速度
  float chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
  float chassis_pitch; //陀螺仪和云台电机叠加的pitch角度
  float chassis_roll;  //陀螺仪和云台电机叠加的roll角度
} chassis_move_t;

/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
extern void chassis_task(void const *pvParameters);

#endif
