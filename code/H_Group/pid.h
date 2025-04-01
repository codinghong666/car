#ifndef __PID_H_
#define __PID_H_

#include "config.h"

//PD控制器
typedef struct PD_Register
{
    float kp;//kp参数
    float kd;//kd参数
    float ki;//ki参数
    float i_add;//积分累计
    float i_th;//积分分离
    float e1;//上一次的误差
    float limit;//输出限幅
    float output;//输出值
}PD_Register;

//PD控制器
typedef struct ADJ_Register
{
    float max;//最大值
    float min;//最小值
    float out;//输出值
}ADJ_Register;
//纯I控制器
typedef struct I_Register
{
    float ki;//ki设定值
    float limit;//输出限幅
    float output;//输出
}I_Register;


// PID控制器需要的全局变量组合
typedef struct PID_Register
{
    volatile unsigned long int timestamp_prev; // 上次时间记忆
    float P;                                   // Kp参数
    float I;                                   // Ki参数
    float D;                                   // Kd参数
    float I_Error;                             // 积分分离阈值
    float I_limit;                             // 积分限幅
    float output_ramp;                         // 输出加速度限幅
    float limit;                               // 输出限幅
    float error_prev;                          // 上次误差记忆
    float output_prev;                         // 上次输出记忆
    float integral_prev;                       // 上次积分记忆
} PID_Register;

typedef struct IPID
{
    float ek_1;   // 上次误差
    float ek_2;   // 上上次误差
    float kp;     // 设定p
    float ki;     // 设定i
    float kd;     // 设定d
    float ei;     // 积分分离参数
    float eo;     // 死区控制参数
    float limit;  // 输出限幅
    float output; // 最终输出
} IPID;

typedef struct FPID
{
    float ek_1;   // 上次误差
    float kp;     // 主要参数
    float kp2;    // 平方参数
    float kd;     // 微分项
    float kd2;    // 陀螺仪微分项
    float limit;  // 输出限幅
    float output; // 最终输出
} FPID;

void PD_Ctrl(float error, PD_Register *p);
void ADJ_PID_Register(u8 state, ADJ_Register *r);
void I_Ctrl(float error, I_Register *p);

u32 Micros(void);
void IPID_Ctrl(float ek, IPID *p);            // 增量PID计算，带积分分离和死区控制
u16 Read_Timer_Cnt(void);                     // 读取定时器的值
void PWM_Transform_Timer_Init(void);          // 初始化PWMB为1us定时器
float PID_Ctrl(float error, PID_Register *p); // 输入偏差量，然后得到
float Fast_PID(float error, FPID *p);
#endif