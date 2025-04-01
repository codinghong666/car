#ifndef __PID_H_
#define __PID_H_

#include "config.h"

//PD������
typedef struct PD_Register
{
    float kp;//kp����
    float kd;//kd����
    float ki;//ki����
    float i_add;//�����ۼ�
    float i_th;//���ַ���
    float e1;//��һ�ε����
    float limit;//����޷�
    float output;//���ֵ
}PD_Register;

//PD������
typedef struct ADJ_Register
{
    float max;//���ֵ
    float min;//��Сֵ
    float out;//���ֵ
}ADJ_Register;
//��I������
typedef struct I_Register
{
    float ki;//ki�趨ֵ
    float limit;//����޷�
    float output;//���
}I_Register;


// PID��������Ҫ��ȫ�ֱ������
typedef struct PID_Register
{
    volatile unsigned long int timestamp_prev; // �ϴ�ʱ�����
    float P;                                   // Kp����
    float I;                                   // Ki����
    float D;                                   // Kd����
    float I_Error;                             // ���ַ�����ֵ
    float I_limit;                             // �����޷�
    float output_ramp;                         // ������ٶ��޷�
    float limit;                               // ����޷�
    float error_prev;                          // �ϴ�������
    float output_prev;                         // �ϴ��������
    float integral_prev;                       // �ϴλ��ּ���
} PID_Register;

typedef struct IPID
{
    float ek_1;   // �ϴ����
    float ek_2;   // ���ϴ����
    float kp;     // �趨p
    float ki;     // �趨i
    float kd;     // �趨d
    float ei;     // ���ַ������
    float eo;     // �������Ʋ���
    float limit;  // ����޷�
    float output; // �������
} IPID;

typedef struct FPID
{
    float ek_1;   // �ϴ����
    float kp;     // ��Ҫ����
    float kp2;    // ƽ������
    float kd;     // ΢����
    float kd2;    // ������΢����
    float limit;  // ����޷�
    float output; // �������
} FPID;

void PD_Ctrl(float error, PD_Register *p);
void ADJ_PID_Register(u8 state, ADJ_Register *r);
void I_Ctrl(float error, I_Register *p);

u32 Micros(void);
void IPID_Ctrl(float ek, IPID *p);            // ����PID���㣬�����ַ������������
u16 Read_Timer_Cnt(void);                     // ��ȡ��ʱ����ֵ
void PWM_Transform_Timer_Init(void);          // ��ʼ��PWMBΪ1us��ʱ��
float PID_Ctrl(float error, PID_Register *p); // ����ƫ������Ȼ��õ�
float Fast_PID(float error, FPID *p);
#endif