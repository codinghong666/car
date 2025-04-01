#ifndef __UART_H_
#define __UART_H_

#include "config.h"

void UART_Init(void); //串口初始化，启动DMA
extern bit uart_dat_pack_done_flag;
extern u8 xdata Ctrl[8];
void DMA_Send_Start(void);
void Send_Byte(char s);
void Uart1_Init(void);

extern volatile u8 xdata  RX_Buffer[20]; // 接收缓冲
extern volatile u8 xdata  TX1_Buffer[30]; // 发送缓冲
extern u16 Tx_Cnt;
#endif