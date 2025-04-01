#ifndef __CONFIG_H_
#define __CONFIG_H_

#if !defined(__C251__) && !defined(__C51__)
#include "../debug.h"
#endif

// 包含官方头文件
#include "STC32F.h"

// 包含基础C支持代码
#include "intrins.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

// 基础功能定义
#define u8 unsigned char
#define u16 unsigned int
#define u32 unsigned long

#define MAIN_Fosc 52000000UL // 定义主时钟
#define Baudrate (65536UL - (MAIN_Fosc / 4) / 200000UL)

extern float ax, ay, az;
extern float gx, gy, gz;
extern float qi, qj, qk, qr;
extern float roll, pitch, yaw;

#define true_state 1
#define false_state 2

#define key1 P54
#define key2 P13
#define key3 P15
#define key4 P14
#define key5 P03
#define buzz P02
#define Y1 P26
#define Y2 P27
#define Y5 P23
#define Y6 P22
#define R1 P21
#define R2 P20
#define R3 P37
#define R4 P36

#define state_noaction 0 //无状态，所有灯灭
#define state_left 1//左转，可以被2次计数器或者直道特征打断，存在两种亮度，根据模式判别
#define state_right 2//右转，可以被2次计数器或者直道特征打断，存在两种亮度，根据模式判别
#define state_double 3//双闪，固定低亮度
#define state_auto_norun 4//自动模式未发车，刹车灯低亮度常亮
#define state_stop 5//刹车灯，可以被加速或者1s计时器关闭

void ICacheOff(void); // 关闭cache高速缓存
void ICacheOn(void);  // 打开cache高速缓存

extern bit long_time_flag;
extern int long_time_cnt;

// 通讯地址定义
#define User_Can_ID 15 // 滤波器设置地址，除这个地址外不在允许接收其他地址的数据
#define All_Can_ID 0   // 作为广播地址存在，不可修改

// 低通滤波器需要的全局变量组合
typedef struct LowPassConfig
{
    volatile unsigned long int last_time; // 上次时间记忆
    float last_result;                    // 上次结果记忆
} LowPassConfig;
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
// 定义四元数结构体
typedef struct Quaternion{
    float w, x, y, z; // 实部r，虚部i,j,k对应于单位四元数ijk
} Quaternion;

void Delay_ms(u16 time);

#if (MAIN_Fosc < 30000000UL)
#define WAIT 0x00
#elif (MAIN_Fosc < 60000000UL)
#define WAIT 0x01
#else
#define WAIT 0x02
#endif

// 自动追频定义
// 定义IRCBAND（IRC频段选择）寄存器设置值
#if (MAIN_Fosc > 35000000UL)
#define BAND 0x02
#elif (MAIN_Fosc > 20000000UL)
#define BAND 0x01
#else
#define BAND 0x00
#endif
#define BAND_MASK 0x03

// 定义CLKDIV寄存器设置值（系统时钟主分频）
#if (MAIN_Fosc > 15000000UL)
#define DIV 0x01
#elif (MAIN_Fosc > 12000000UL)
#define DIV 0x02
#elif (MAIN_Fosc > 8000000UL)
#define DIV 0x03
#elif (MAIN_Fosc > 6000000UL)
#define DIV 0x04
#else
#define DIV 0x05
#endif

// 定义追频寄存器目标值
#define MCLK (MAIN_Fosc * DIV)

#if (MCLK < 50000000UL)
#define CNT ((16 * MCLK) / 32768) // 低频
#define CREHF 0x00
#else
#define CNT ((8 * MCLK) / 32768) // 高频
#define CREHF 0x08
#endif

#define XRES (CNT * 5 / 1000)

#endif
