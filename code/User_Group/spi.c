#include "./H_Group/spi.h" //w25q128���


void SPI_Init(void)
{
    // P_SW2 |= 0x80;                             // ��չ�Ĵ���(XFR)����ʹ��
    // SPSTAT = 0xc0;                             // �����־λ
    // SPCTL = 0xd7;                              // ����SS���Ź��ܣ�ʹ��SPI���ܣ�MSB���ȣ�����ģʽ,���е͵�ƽ
    // ESPI = 1;
    // DMA_SPI_STA = 0x00;                        // ����DMA��־λ
    // DMA_SPI_CFG2 = 0x03;                       // ���Զ�����SS,SS����ΪP3.5
    // DMA_SPI_CFG = 0xc5;                        // ʹ��SPI_DMA�жϣ������ͣ���ֹ���ܡ��ж����ȼ�0�����߷�������0
    // DMA_SPI_CR = 0x81;                         // ����SPI_DMA���ܣ���ʼ����ǰ���FIFO
    // DMA_SPI_RXAH = (u8)((u16)&Led_TxBuf >> 8); // SPI�������ݴ洢��ַ
    // DMA_SPI_RXAL = (u8)((u16)&Led_TxBuf);
    // DMA_SPI_AMT = Buf_Lenth;
    // HSSPI_CFG2 |= 0x20; // ��������SPIģʽ
    // CLKSEL |= 0x40;     // ѡ��HPLL��ΪHSCLK
    // HSCLKDIV = 39;      // �����ȷ300usʱ��
}

void SPI_Isr(void) interrupt 9
{
		SPSTAT = 0xc0;                             // �����־λ
}

void SPI_DMA_Isr(void) interrupt 49
{
  DMA_SPI_STA = 0; // ����жϱ�־λ
}