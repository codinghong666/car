#include "./H_Group/spi.h" //w25q128相关


void SPI_Init(void)
{
    // P_SW2 |= 0x80;                             // 扩展寄存器(XFR)访问使能
    // SPSTAT = 0xc0;                             // 清除标志位
    // SPCTL = 0xd7;                              // 忽略SS引脚功能，使能SPI功能，MSB优先，主机模式,空闲低电平
    // ESPI = 1;
    // DMA_SPI_STA = 0x00;                        // 清零DMA标志位
    // DMA_SPI_CFG2 = 0x03;                       // 不自动控制SS,SS引脚为P3.5
    // DMA_SPI_CFG = 0xc5;                        // 使能SPI_DMA中断，允许发送，禁止接受。中断优先级0，总线访问优先0
    // DMA_SPI_CR = 0x81;                         // 允许SPI_DMA功能，开始操作前清空FIFO
    // DMA_SPI_RXAH = (u8)((u16)&Led_TxBuf >> 8); // SPI发送数据存储地址
    // DMA_SPI_RXAL = (u8)((u16)&Led_TxBuf);
    // DMA_SPI_AMT = Buf_Lenth;
    // HSSPI_CFG2 |= 0x20; // 启动高速SPI模式
    // CLKSEL |= 0x40;     // 选择HPLL作为HSCLK
    // HSCLKDIV = 39;      // 输出精确300us时钟
}

void SPI_Isr(void) interrupt 9
{
		SPSTAT = 0xc0;                             // 清除标志位
}

void SPI_DMA_Isr(void) interrupt 49
{
  DMA_SPI_STA = 0; // 清除中断标志位
}