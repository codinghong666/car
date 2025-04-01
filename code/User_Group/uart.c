#include "./H_Group/uart.h"

// ���ڲ��ֶ���
u8 xdata Ctrl[8] = {0};
volatile u8 xdata  RX_Buffer[20] = {0}; // ���ջ���
volatile u8 xdata  TX1_Buffer[30] = {0}; // ���ͻ���
u16 Tx_Cnt = 0;                              // ���ͼ���ָ��
bit B_TX_Busy = 0;                           // ����æ��־



void DMA_Send_Start(void) // ����һ��DMA���ڷ���
{
    if (Tx_Cnt < 100 && Tx_Cnt > 0)
    {
        DMA_UR1T_AMT = Tx_Cnt - 1; // �趨�����ܳ�
        DMA_UR1T_AMTH = ((Tx_Cnt - 1) >> 8);
        Tx_Cnt = 0;                // �������ָ��
        if (DMA_UR1T_STA == 0)
            DMA_UR1T_CR = 0xc0; // ����һ�η���
    }
    else
        Tx_Cnt = 0;
}

void UART1T_DMA_Isr(void) interrupt DMA_UR1T_VECTOR
{
    DMA_UR1T_STA = 0x00; // �����־λ
}

void Uart1_Isr(void) interrupt 4
{
	if (TI)				//��⴮��1�����ж�
	{
		TI = 0;			//�������1�����ж�����λ
        B_TX_Busy = 0;
	}
	if (RI)				//��⴮��1�����ж�
	{
		RI = 0;			//�������1�����ж�����λ
	}
}

void Uart1_Init(void)	//115200bps@52MHz
{
	SCON |= 0xc0;		//8λ����,�ɱ䲨����
	AUXR |= 0x40;		//��ʱ��ʱ��1Tģʽ
	AUXR &= 0xFE;		//����1ѡ��ʱ��1Ϊ�����ʷ�����
	TMOD &= 0x0F;		//���ö�ʱ��ģʽ
	TL1 = 0x8F;			//���ö�ʱ��ʼֵ
	TH1 = 0xFF;			//���ö�ʱ��ʼֵ
	ET1 = 0;			//��ֹ��ʱ���ж�
	TR1 = 1;			//��ʱ��1��ʼ��ʱ
	ES = 1;				//ʹ�ܴ���1�ж�
    USARTCR2 |= 0x04;
    // DMA���ڲ������Ӳ���
    DMA_UR1T_CFG = 0x80; // ���������ж�
    DMA_UR1T_STA = 0x00; // ���״̬��־λ
    DMA_UR1T_AMT = 0;  // ����д��û�ã����淢��֮ǰ�����¸�����
    DMA_UR1T_TXAH = (u8)((u16)&TX1_Buffer >> 8);
    DMA_UR1T_TXAL = (u8)((u16)&TX1_Buffer);
    DMA_UR1T_CR = 0xc0;
}



void UART_Init(void) // ʹ��Timer2��������
{
    P_SW2 |= 0x80;
    // ��ʱ��2��200kbps������@52
    S2CON = 0x50; // 8λ����,�ɱ䲨����
    S2CFG = 0x01;
    AUXR |= 0x04; // ��ʱ��ʱ��1Tģʽ
    T2L = 0xBF;   // ���ö�ʱ��ʼֵ
    T2H = 0xFF;   // ���ö�ʱ��ʼֵ
    AUXR |= 0x10; // ��ʱ��2��ʼ��ʱ
    IE2 |= 0x01;  // ʹ�ܴ���2�ж�

    UR2TOCR = 0x60; // �رճ�ʱ���ܣ���ʱ����ʱ��Դѡ���ڲ�ʱ�ӣ�ʹ�ܳ�ʱ�ж�
    UR2TOTH = (u8)(26000 >> 8);
    UR2TOTL = (u8)(26000);

    // DMA���ڲ������Ӳ���
    // ���ղ���һֱ�򿪣����յ����ݣ����ҳ�ʱ���ٽ��в���
    DMA_UR2R_CFG = 0x8f; // �ж����ȼ����
    DMA_UR2R_STA = 0x00; // ���״̬��־λ
    DMA_UR2R_AMT = 16;   // ���һ��16������,��ʱ�Զ��ж�
    DMA_UR2R_RXAH = (u8)((u16)&RX_Buffer >> 8);
    DMA_UR2R_RXAL = (u8)((u16)&RX_Buffer);
    DMA_UR2R_CR = 0xa1; // ���FIFO,��������
    UR2TOSR = 0x00;     // ����жϱ�־λ
}

u8 test = 0;
u8 i;
bit uart_dat_pack_done_flag = 0;
void Uart2_Isr(void) interrupt 8
{
    if (S2CON & 0x02) // ��⴮��2�����ж�
    {
        S2CON &= ~0x02; // �������2�����ж�����λ
    }
    if (S2CON & 0x01) // ��⴮��2�����ж�
    {
        UR2TOCR |= 0x80; // ʹ�ܳ�ʱ����
        S2CON &= ~0x01;  // �������2�����ж�����λ
        UR2TOSR = 0x00;  // ����жϱ�־λ,��λ��ʱ������
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
        UR2TOCR &= ~0x80; // �رճ�ʱ�жϹ���
        UR2TOSR = 0x00;   // ����жϱ�־λ
        DMA_UR2R_STA = 0x00;
        DMA_UR2R_CR = 0x01;  // �ر�DMA���ܣ��������Fifo
        DMA_UR2R_CR |= 0xa0; // ��DMA���ܣ�����һ������
    }
}

void UART2R_DMA_Isr(void) interrupt 53
{
    DMA_UR2R_STA = 0x00;
}