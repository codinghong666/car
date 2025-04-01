#include "./H_Group/uart.h"

// 串口部分定义
u8 xdata Ctrl[8] = {0};
volatile u8 xdata  RX_Buffer[20] = {0}; // 接收缓冲
volatile u8 xdata  TX1_Buffer[30] = {0}; // 发送缓冲
u16 Tx_Cnt = 0;                              // 发送计数指针
bit B_TX_Busy = 0;                           // 发送忙标志



void DMA_Send_Start(void) // 启动一次DMA串口发送
{
    if (Tx_Cnt < 100 && Tx_Cnt > 0)
    {
        DMA_UR1T_AMT = Tx_Cnt - 1; // 设定发送总长
        DMA_UR1T_AMTH = ((Tx_Cnt - 1) >> 8);
        Tx_Cnt = 0;                // 清除发送指针
        if (DMA_UR1T_STA == 0)
            DMA_UR1T_CR = 0xc0; // 启动一次发送
    }
    else
        Tx_Cnt = 0;
}

void UART1T_DMA_Isr(void) interrupt DMA_UR1T_VECTOR
{
    DMA_UR1T_STA = 0x00; // 清除标志位
}

void Uart1_Isr(void) interrupt 4
{
	if (TI)				//检测串口1发送中断
	{
		TI = 0;			//清除串口1发送中断请求位
        B_TX_Busy = 0;
	}
	if (RI)				//检测串口1接收中断
	{
		RI = 0;			//清除串口1接收中断请求位
	}
}

void Uart1_Init(void)	//115200bps@52MHz
{
	SCON |= 0xc0;		//8位数据,可变波特率
	AUXR |= 0x40;		//定时器时钟1T模式
	AUXR &= 0xFE;		//串口1选择定时器1为波特率发生器
	TMOD &= 0x0F;		//设置定时器模式
	TL1 = 0x8F;			//设置定时初始值
	TH1 = 0xFF;			//设置定时初始值
	ET1 = 0;			//禁止定时器中断
	TR1 = 1;			//定时器1开始计时
	ES = 1;				//使能串口1中断
    USARTCR2 |= 0x04;
    // DMA串口操作附加部分
    DMA_UR1T_CFG = 0x80; // 启动串口中断
    DMA_UR1T_STA = 0x00; // 清除状态标志位
    DMA_UR1T_AMT = 0;  // 这里写了没用，后面发送之前会重新给定的
    DMA_UR1T_TXAH = (u8)((u16)&TX1_Buffer >> 8);
    DMA_UR1T_TXAL = (u8)((u16)&TX1_Buffer);
    DMA_UR1T_CR = 0xc0;
}



void UART_Init(void) // 使用Timer2做波特率
{
    P_SW2 |= 0x80;
    // 定时器2，200kbps波特率@52
    S2CON = 0x50; // 8位数据,可变波特率
    S2CFG = 0x01;
    AUXR |= 0x04; // 定时器时钟1T模式
    T2L = 0xBF;   // 设置定时初始值
    T2H = 0xFF;   // 设置定时初始值
    AUXR |= 0x10; // 定时器2开始计时
    IE2 |= 0x01;  // 使能串口2中断

    UR2TOCR = 0x60; // 关闭超时功能，超时计数时钟源选择内部时钟，使能超时中断
    UR2TOTH = (u8)(26000 >> 8);
    UR2TOTL = (u8)(26000);

    // DMA串口操作附加部分
    // 接收部分一直打开，接收到数据，并且超时后再进行操作
    DMA_UR2R_CFG = 0x8f; // 中断优先级最高
    DMA_UR2R_STA = 0x00; // 清除状态标志位
    DMA_UR2R_AMT = 16;   // 最多一次16个数据,超时自动中断
    DMA_UR2R_RXAH = (u8)((u16)&RX_Buffer >> 8);
    DMA_UR2R_RXAL = (u8)((u16)&RX_Buffer);
    DMA_UR2R_CR = 0xa1; // 清空FIFO,并且启动
    UR2TOSR = 0x00;     // 清空中断标志位
}

u8 test = 0;
u8 i;
bit uart_dat_pack_done_flag = 0;
void Uart2_Isr(void) interrupt 8
{
    if (S2CON & 0x02) // 检测串口2发送中断
    {
        S2CON &= ~0x02; // 清除串口2发送中断请求位
    }
    if (S2CON & 0x01) // 检测串口2接收中断
    {
        UR2TOCR |= 0x80; // 使能超时功能
        S2CON &= ~0x01;  // 清除串口2接收中断请求位
        UR2TOSR = 0x00;  // 清空中断标志位,复位超时计数器
    }
    if (UR2TOSR & 0x01)
    {
        test = DMA_UR2R_DONE;
        if (DMA_UR2R_DONE == 8)
        {
            uart_dat_pack_done_flag = 1;
            for (i = 0; i < 8; i++)
            {
                Ctrl[i] = RX_Buffer[i];
            }
            uart_dat_pack_done_flag = 0;
        }
        UR2TOCR &= ~0x80; // 关闭超时中断功能
        UR2TOSR = 0x00;   // 清空中断标志位
        DMA_UR2R_STA = 0x00;
        DMA_UR2R_CR = 0x01;  // 关闭DMA功能，清除内置Fifo
        DMA_UR2R_CR |= 0xa0; // 打开DMA功能，触发一次启动
    }
}

void UART2R_DMA_Isr(void) interrupt 53
{
    DMA_UR2R_STA = 0x00;
}