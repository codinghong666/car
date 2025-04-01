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
    PWMA_PS = 0x00;    // 高级 PWM 通道输出脚选择位
    PWMA_PS |= PWM1_2; // 选择 PWM1_2 通道

    PWMA_ENO = 0x00;
    PWMA_ENO |= ENO1P; // 使能输出，小脉冲

    PWMA_CNTRH = 0x00;
    PWMA_CNTRL = 0x00;

    PWMA_ARRH = (u8)(set_period >> 8); // 设置周期时间
    PWMA_ARRL = (u8)set_period;

    PWMA_CCR1H = (u8)(set_ccr >> 8);
    PWMA_CCR1L = (u8)(set_ccr);

    PWMA_CR2 = 0x30; // 设置比较捕获打开ADC

    PWMA_CCER1 = 0x00; // 写 CCMRx 前必须先清零 CCxE 关闭通道
    PWMA_CCMR1 = 0x60; // 通道模式配置
    PWMA_CCER1 = 0x01; // 配置通道输出使能和极性

    PWMA_BKR = 0x80;  // 使能主输出
    PWMA_CR1 |= 0x01; // 使能计数器

    ADC_CONTR = 12; // 固定打开12通道
    ADC_POWER = 1;
    ADC_EPWMT = 1;
    ADC_FLAG = 0;
    ADCCFG = 0x20;         // 调整为右对齐，ADC时钟速度最快
    ADCTIM = 0x9f;         // 最大采样时间
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

u16 ADC_Cnt = 0;    // ADC缓冲计数
bit ADC_Finish = 0; // 成功置1
void Start_Get_Image(void)
{
    PWMA_CR1 &= ~0x01; // 暂时关闭PWMA
    PWMA_CNTRH = 0x00;
    PWMA_CNTRL = 0x00; // 计数清零
    PWMA_ENO = 0x00;   // 关闭PWMA端口输出
    CCD_CLK = 1;
    CCD_SI = 0;
    CCD_CLK = 0;
    CCD_SI = 1;
    CCD_CLK = 1;
    CCD_SI = 0;
    CCD_CLK = 0;
    ADC_START = 1;
    PWMA_ENO |= ENO1P; // 使能输出，小脉冲
    ADC_Cnt = 0;       // 从零开始计数
    PWMA_CR1 |= 0x01;  // 启动PWMA定时器计数
    ADC_Finish = 0;
}

void ADC_Isr(void) interrupt 5
{
    ADC_FLAG = 0;
    adc_tmp[ADC_Cnt] = ((u16)ADC_RES << 8 | ADC_RESL);
    if (ADC_Cnt >= 128)
    {
        PWMA_CR1 &= ~0x01; // 暂时关闭PWMA
        ADC_Finish = 1;
    }else
    {
        ADC_Cnt++;
    }
}