/**
 * @file shoot_task.h
 * @author 何清华
 * @brief 包含对摩擦轮以及拨盘的控制
 * @version 0.1
 * @date 2022-03-23
 *
 * @copyright Copyright (c) 2022
 *
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __shoot_TASK_H__
#define __shoot_TASK_H__

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"


//任务初始化 空闲一段时间
#define SHOOT_TASK_INIT_TIME        300
//射击任务控制任务时间函 5ms
#define SHOOT_CONTROL_TIME          5

//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       RIGHT_SWITCH

//鼠标长按判断 100 * 5 = 500ms
#define PRESS_LONG_TIME             100
//遥控器射击开关开启一段时间后 连续发射子弹 用于清弹 400 * 5 = 2000ms
#define RC_S_LONG_TIME              400

//发射机构pid参数
//摩擦轮
#define FRICTION_KP 30.0f
#define FRICTION_KD 0.0f
#define FRICTION_KI 0.0f
#define FRICTION_MAX_OUT 7000.0f//5000.0f//6000.f33
#define FRICTION_MAX_IOUT 6000.0f
//拨盘
//2006电机拨弹盘的pid
#define TRIGGER_KP     19.0f//21.0f//22.0f  //20.5f //20.0f    //6.0f//20.0f    //120.0f
#define TRIGGER_KI     0.0f//0.5f
#define TRIGGER_KD     0.1f//0.1f//0//1.0f
#define TRIGGER_MAX_OUT  9500.0f//10000.0f
#define TRIGGER_MAX_IOUT 6000.0f


//拨弹速度
#define TRIGGER_SPEED               1000.0f//350.0f//单发转速
#define CONTINUE_TRIGGER_SPEED      2300.0f//2500.0f//4500.0f//2500.0f//500.0f//连发转速
#define READY_TRIGGER_SPEED         0.0f//勿改```````````````````````````````````````````````````````````````````````````````

/*****************************摩擦轮速度*****************************/
/*****************************摩擦轮速度*****************************/
/*****************************摩擦轮速度*****************************/

#define FRICTION_SPEED_SET    6800.0f//5500  //7000//4250//7000  //比赛4250 //4000  //4055.0f 

/*****************************摩擦轮速度*****************************/
/*****************************摩擦轮速度*****************************/
/*****************************摩擦轮速度*****************************/
/*****************************摩擦轮速度*****************************/
#define FRICTION_ACCEL_MAX_OUT      1000//摩擦轮启动加速电流限幅

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//卡弹时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  400 //5 * 400 = 2000ms 
#define REVERSE_TIME                150 //5 * 150 = 750ms
#define REVERSE_SPEED_LIMIT         13.0f

//热量预留值，防止超热量
#define SHOOT_HEAT_REMAIN_VALUE     40//200//100  //80

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
} shoot_mode_e;

typedef struct
{
    const motor_measure_t *motor_measure;
    pid_type_def motor_speed_pid;
    float speed;
    float speed_set;
    float angle;
    float set_angle;
    int16_t given_current;
    int8_t ecd_count;
} trigger_motor_t;

typedef struct
{
    const motor_measure_t *motor_measure;
    pid_type_def motor_speed_pid;
    float accel;          //加速度
    float speed;          //当前速度
    float speed_set;      //设定速度
    int16_t given_current; //发送电流
} friction_motor_t;

typedef struct
{
    const motor_measure_t *motor_measure;
    pid_type_def motor_speed_pid;
    float accel;          //加速度
    float speed;          //当前速度
    float speed_set;      //设定速度
    int16_t given_current; //发送电流
} avoid_shoot_motor_t;


typedef struct
{
    shoot_mode_e shoot_mode;

    trigger_motor_t trigger_motor;
    friction_motor_t right_fricition_motor;
    friction_motor_t left_fricition_motor;
	  avoid_shoot_motor_t avoid_shoot_motor;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
	

    uint16_t heat_limit;
    uint16_t heat;
} shoot_control_t;

void fire_task(void const *pvParameters);

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
