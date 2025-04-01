#include "pwm_adc.h"

#define PWM1_1 0x00 // P:P1.0  N:P1.1
#define PWM1_2 0x01 // P:P2.0  N:P2.1
#define PWM1_3 0x02 // P:P6.0  N:P6.1

#define PWM2_1 0x00 // P:P1.2/P5.4  N:P1.3
#define PWM2_2 0x04 // P:P2.2  N:P2.3
#define PWM2_3 0x08 // P:P6.2  N:P6.3

#define PWM3_1 0x00 // P:P1.4  N:P1.5
#define PWM3_2 0x10 // P:P2.4  N:P2.5
#define PWM3_3 0x20 // P:P6.4  N:P6.5

#define PWM4_1 0x00 // P:P1.6  N:P1.7
#define PWM4_2 0x40 // P:P2.6  N:P2.7
#define PWM4_3 0x80 // P:P6.6  N:P6.7
#define PWM4_4 0xC0 // P:P3.4  N:P3.3

#define ENO1P 0x01
#define ENO1N 0x02
#define ENO2P 0x04
#define ENO2N 0x08
#define ENO3P 0x10
#define ENO3N 0x20
#define ENO4P 0x40
#define ENO4N 0x80

u16 set_period = 1300;
u16 set_ccr = 100;

void PWM_ADC_Init(void)
{
    PWMA_PS = 0x00;    // �߼� PWM ͨ�������ѡ��λ
    PWMA_PS |= PWM1_2; // ѡ�� PWM1_2 ͨ��

    PWMA_ENO = 0x00;
    PWMA_ENO |= ENO1P; // ʹ�������С����

    PWMA_CNTRH = 0x00;
    PWMA_CNTRL = 0x00;

    PWMA_ARRH = (u8)(set_period >> 8); // ��������ʱ��
    PWMA_ARRL = (u8)set_period;

    PWMA_CCR1H = (u8)(set_ccr >> 8);
    PWMA_CCR1L = (u8)(set_ccr);

    PWMA_CR2 = 0x30; // ���ñȽϲ����ADC

    PWMA_CCER1 = 0x00; // д CCMRx ǰ���������� CCxE �ر�ͨ��
    PWMA_CCMR1 = 0x60; // ͨ��ģʽ����
    PWMA_CCER1 = 0x01; // ����ͨ�����ʹ�ܺͼ���

    PWMA_BKR = 0x80;  // ʹ�������
    PWMA_CR1 |= 0x01; // ʹ�ܼ�����

    ADC_CONTR = 12; // �̶���12ͨ��
    ADC_POWER = 1;
    ADC_EPWMT = 1;
    ADC_FLAG = 0;
    ADCCFG = 0x20;         // ����Ϊ�Ҷ��룬ADCʱ���ٶ����
    ADCTIM = 0x9f;         // ������ʱ��
    EADC = 1;
    PWMA_IER = 0x01;
}

void PWMA_Isr(void) interrupt 26
{
    if (PWMA_SR1 & 0x01)
    {
        PWMA_SR1 &= ~0x01;
    }
}

#define CCD_AO P04  // ADC12
#define CCD_CLK P20 // PWM1P
#define CCD_SI P02

u16 ADC_Cnt = 0;    // ADC�������
bit ADC_Finish = 0; // �ɹ���1
void Start_Get_Image(void)
{
    PWMA_CR1 &= ~0x01; // ��ʱ�ر�PWMA
    PWMA_CNTRH = 0x00;
    PWMA_CNTRL = 0x00; // ��������
    PWMA_ENO = 0x00;   // �ر�PWMA�˿����
    CCD_CLK = 1;
    CCD_SI = 0;
    CCD_CLK = 0;
    CCD_SI = 1;
    CCD_CLK = 1;
    CCD_SI = 0;
    CCD_CLK = 0;
    ADC_START = 1;
    PWMA_ENO |= ENO1P; // ʹ�������С����
    ADC_Cnt = 0;       // ���㿪ʼ����
    PWMA_CR1 |= 0x01;  // ����PWMA��ʱ������
    ADC_Finish = 0;
}

void ADC_Isr(void) interrupt 5
{
    ADC_FLAG = 0;
    adc_tmp[ADC_Cnt] = ((u16)ADC_RES << 8 | ADC_RESL);
    if (ADC_Cnt >= 128)
    {
        PWMA_CR1 &= ~0x01; // ��ʱ�ر�PWMA
        ADC_Finish = 1;
    }else
    {
        ADC_Cnt++;
    }
}