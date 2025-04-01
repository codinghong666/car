#ifndef __CONFIG_H_
#define __CONFIG_H_

#if !defined(__C251__) && !defined(__C51__)
#include "../debug.h"
#endif

// �����ٷ�ͷ�ļ�
#include "STC32F.h"

// ��������C֧�ִ���
#include "intrins.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

// �������ܶ���
#define u8 unsigned char
#define u16 unsigned int
#define u32 unsigned long

#define MAIN_Fosc 52000000UL // ������ʱ��
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

#define state_noaction 0 //��״̬�����е���
#define state_left 1//��ת�����Ա�2�μ���������ֱ��������ϣ������������ȣ�����ģʽ�б�
#define state_right 2//��ת�����Ա�2�μ���������ֱ��������ϣ������������ȣ�����ģʽ�б�
#define state_double 3//˫�����̶�������
#define state_auto_norun 4//�Զ�ģʽδ������ɲ���Ƶ����ȳ���
#define state_stop 5//ɲ���ƣ����Ա����ٻ���1s��ʱ���ر�

void ICacheOff(void); // �ر�cache���ٻ���
void ICacheOn(void);  // ��cache���ٻ���

extern bit long_time_flag;
extern int long_time_cnt;

// ͨѶ��ַ����
#define User_Can_ID 15 // �˲������õ�ַ���������ַ�ⲻ���������������ַ������
#define All_Can_ID 0   // ��Ϊ�㲥��ַ���ڣ������޸�

// ��ͨ�˲�����Ҫ��ȫ�ֱ������
typedef struct LowPassConfig
{
    volatile unsigned long int last_time; // �ϴ�ʱ�����
    float last_result;                    // �ϴν������
} LowPassConfig;
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
// ������Ԫ���ṹ��
typedef struct Quaternion{
    float w, x, y, z; // ʵ��r���鲿i,j,k��Ӧ�ڵ�λ��Ԫ��ijk
} Quaternion;

void Delay_ms(u16 time);

#if (MAIN_Fosc < 30000000UL)
#define WAIT 0x00
#elif (MAIN_Fosc < 60000000UL)
#define WAIT 0x01
#else
#define WAIT 0x02
#endif

// �Զ�׷Ƶ����
// ����IRCBAND��IRCƵ��ѡ�񣩼Ĵ�������ֵ
#if (MAIN_Fosc > 35000000UL)
#define BAND 0x02
#elif (MAIN_Fosc > 20000000UL)
#define BAND 0x01
#else
#define BAND 0x00
#endif
#define BAND_MASK 0x03

// ����CLKDIV�Ĵ�������ֵ��ϵͳʱ������Ƶ��
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

// ����׷Ƶ�Ĵ���Ŀ��ֵ
#define MCLK (MAIN_Fosc * DIV)

#if (MCLK < 50000000UL)
#define CNT ((16 * MCLK) / 32768) // ��Ƶ
#define CREHF 0x00
#else
#define CNT ((8 * MCLK) / 32768) // ��Ƶ
#define CREHF 0x08
#endif

#define XRES (CNT * 5 / 1000)

#endif
