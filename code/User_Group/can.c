#include "./H_Group/can.h"
bit B_Can1Send = 0;
bit B_Can1Read = 0;

CAN_DataDef CAN1_Tx;
CAN_DataDef CAN1_Rx[8];

unsigned char CanReadReg(unsigned char addr)
{
    CANAR = addr;
    return CANDR;
}

void CanWriteReg(unsigned char addr, unsigned char dat)
{
    CANAR = addr;
    CANDR = dat;
}

void CanSetBaudrate(void) // 500Kbps@52MHz
{
    CanWriteReg(MR, 0x04);   // ʹ��Resetģʽ
    CanWriteReg(BTR0, 0xc3); // SJW(3), BRP(3)
    CanWriteReg(BTR1, 0xc6); // SAM(1), TSG2(4), TSG1(6)
    CanWriteReg(MR, 0x00);   // �˳�Resetģʽ
}

void CanReadFifo(CAN_DataDef *CAN)
{
    u8 i;
    u8 pdat[5];
    u8 RX_Index = 0;

    pdat[0] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3)));

    if (pdat[0] & 0x80) // �ж��Ǳ�׼֡������չ֡
    {
        pdat[1] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3))); // ��չ֡IDռ4���ֽ�
        pdat[2] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3)));
        pdat[3] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3)));
        pdat[4] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3)));
        CAN->ID = (((u32)pdat[1] << 24) + ((u32)pdat[2] << 16) + ((u32)pdat[3] << 8) + pdat[4]) >> 3;
    }
    else
    {
        pdat[1] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3))); // ��׼֡IDռ2���ֽ�
        pdat[2] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3)));
        CAN->ID = ((pdat[1] << 8) + pdat[2]) >> 5;
    }

    CAN->FF = pdat[0] >> 7;  // ֡��ʽ
    CAN->RTR = pdat[0] >> 6; // ֡����
    CAN->DLC = pdat[0];      // ���ݳ���

    for (i = 0; ((i < CAN->DLC) && (i < 8)); i++) // ��ȡ���ݳ���Ϊlen����಻����8
    {
        CAN->DataBuffer[i] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3))); // ��ȡ��Ч����
    }
    while (RX_Index & 3) // �ж��Ѷ����ݳ����Ƿ�4��������
    {
        CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3))); // ��ȡ������ݣ�һ֡����ռ��4���������������ռ䣬���㲹0
    }
}

u8 CanReadMsg(CAN_DataDef *CAN)
{
    u8 i;
    u8 n = 0;
    do
    {
        CanReadFifo(&CAN[n++]); // ��ȡ���ջ���������
        i = CanReadReg(SR);
    } while (i & 0x80); // �жϽ��ջ��������Ƿ������ݣ��еĻ�������ȡ

    return n; // ����֡����
}

void CanSendMsg(CAN_DataDef *CAN)
{
    u32 CanID;
    u8 RX_Index, i;

    if (CANSEL == CAN1) // �ж��Ƿ�CAN1
    {
        while (B_Can1Send)
            ; // �ȴ�CAN1�ϴη������
    }

    if (CAN->FF) // �ж��Ƿ���չ֡
    {
        CanID = CAN->ID << 3;
        CanWriteReg(TX_BUF0, CAN->DLC | ((u8)CAN->RTR << 6) | 0x80); // bit7: ��׼֡(0)/��չ֡(1), bit6: ����֡(0)/Զ��֡(1), bit3~bit0: ���ݳ���(DLC)
        CanWriteReg(TX_BUF1, (u8)(CanID >> 24));
        CanWriteReg(TX_BUF2, (u8)(CanID >> 16));
        CanWriteReg(TX_BUF3, (u8)(CanID >> 8));
        CanWriteReg(TX_BUF0, (u8)CanID);

        RX_Index = 1;
        for (i = 0; ((i < CAN->DLC) && (i < 8)); i++) // ���ݳ���ΪDLC����಻����8
        {
            CanWriteReg((u8)(TX_BUF0 + (RX_Index++ & 3)), CAN->DataBuffer[i]); // д����Ч����
        }
        while (RX_Index & 3) // �ж��Ѷ����ݳ����Ƿ�4��������
        {
            CanWriteReg((u8)(TX_BUF0 + (RX_Index++ & 3)), 0x00); // д��������ݣ�һ֡����ռ��4���������������ռ䣬���㲹0
        }
    }
    else // ���ͱ�׼֡
    {
        CanID = (u16)(CAN->ID << 5);
        CanWriteReg(TX_BUF0, CAN->DLC | ((u8)CAN->RTR << 6)); // bit7: ��׼֡(0)/��չ֡(1), bit6: ����֡(0)/Զ��֡(1), bit3~bit0: ���ݳ���(DLC)
        CanWriteReg(TX_BUF1, (u8)(CanID >> 8));
        CanWriteReg(TX_BUF2, (u8)CanID);

        RX_Index = 3;
        for (i = 0; ((i < CAN->DLC) && (i < 8)); i++) // ���ݳ���ΪDLC����಻����8
        {
            CanWriteReg((u8)(TX_BUF0 + (RX_Index++ & 3)), CAN->DataBuffer[i]); // д����Ч����
        }
        while (RX_Index & 3) // �ж��Ѷ����ݳ����Ƿ�4��������
        {
            CanWriteReg((u8)(TX_BUF0 + (RX_Index++ & 3)), 0x00); // д��������ݣ�һ֡����ռ��4���������������ռ䣬���㲹0
        }
    }
    CanWriteReg(CMR, 0x04); // ����һ��֡����

    if (CANSEL == CAN1) // �ж��Ƿ�CAN1
    {
        B_Can1Send = 1; // ����CAN1����æ��־
    }
}

void PortSwitch(void)
{
    P_SW1 &= ~0x30; // CAN1: CANRX(P0.0), CANTX(P0.1)
}

void Can_Init(void)
{
    CANEN = 1;     // ��CAN1ģ��
    CANSEL = CAN1; // ѡ��CAN1ģ��
    PortSwitch();
    CanSetBaudrate();
    CanWriteReg(MR, 0x05);                     // ʹ��Resetģʽ����˫�˲�������
    CanWriteReg(ACR0, (u8)(User_Can_ID >> 3)); // �������մ���Ĵ���,�����豸ID�͹㲥ID
    CanWriteReg(ACR1, (u8)(User_Can_ID << 5));
    CanWriteReg(ACR2, 0x00); // �㲥��ַĬ��Ϊ0������Ҫ����
    CanWriteReg(ACR3, 0x00);

    CanWriteReg(AMR0, 0x00); // �����������μĴ���������λ��У�飬RTR��У��
    CanWriteReg(AMR1, 0x1f);
    CanWriteReg(AMR2, 0x00);
    CanWriteReg(AMR3, 0x1f);

    CanWriteReg(ISR, 0xff); // ���жϱ�־
    CanWriteReg(IMR, 0xff); // �жϼĴ�������,��Ӧ�����ж�
    CanWriteReg(MR, 0x00);  // �˳� Reset Mode
    CANICR |= 0x02;         // CAN1�ж�ʹ��
}

void CANBUS1_Interrupt(void) interrupt CAN1_VECTOR
{
    u8 isr;
    u8 store;
    u8 arTemp;

    arTemp = CANAR; // ��CANAR�ֳ����棬������ѭ����д�� CANAR ������жϣ����ж����޸��� CANAR ����
    store = AUXR2;  // ��AUXR2�ֳ�����

    AUXR2 &= ~0x08; // ѡ��CAN1ģ��
    isr = CanReadReg(ISR);

    if ((isr & 0x04) == 0x04) // TI
    {
        CANAR = ISR;
        CANDR = 0x04; // CLR FLAG

        B_Can1Send = 0;
    }
    if ((isr & 0x08) == 0x08) // RI
    {
        CANAR = ISR;
        CANDR = 0x08; // CLR FLAG

        B_Can1Read = 1;
    }

    if ((isr & 0x40) == 0x40) // ALI
    {
        CANAR = ISR;
        CANDR = 0x40; // CLR FLAG
    }

    if ((isr & 0x20) == 0x20) // EWI
    {
        CANAR = MR;
        CANDR &= ~0x04; // ��� Reset Mode, ��BUS-OFF״̬�˳�

        CANAR = ISR;
        CANDR = 0x20; // CLR FLAG
    }

    if ((isr & 0x10) == 0x10) // EPI
    {
        CANAR = ISR;
        CANDR = 0x10; // CLR FLAG
    }

    if ((isr & 0x02) == 0x02) // BEI
    {
        CANAR = ISR;
        CANDR = 0x02; // CLR FLAG
    }

    if ((isr & 0x01) == 0x01) // DOI
    {
        CANAR = ISR;
        CANDR = 0x01; // CLR FLAG
    }

    AUXR2 = store;  // ��AUXR2�ֳ��ָ�
    CANAR = arTemp; // ��CANAR�ֳ��ָ�
}

int camera_dat_l, camera_dat_r;
u8 camera_state_l, camera_state_r;

long read_left_postion, read_right_postion;

void Can_Dat_Handle(CAN_DataDef *can_dat)
{
    if (can_dat->DLC == 5) // �ж�Ϊ���ص��ֱ�����
    {
        if (can_dat->ID == User_Can_ID)
        {
            if (can_dat->DataBuffer[4] == 3) // ����
            {
                read_left_postion = (long)((u32)can_dat->DataBuffer[0] << 24 | (u32)can_dat->DataBuffer[1] << 16 | (u32)can_dat->DataBuffer[2] << 8 | (u32)can_dat->DataBuffer[3]);
            }
            if (can_dat->DataBuffer[4] == 2) // ����
            {
                read_right_postion = (long)((u32)can_dat->DataBuffer[0] << 24 | (u32)can_dat->DataBuffer[1] << 16 | (u32)can_dat->DataBuffer[2] << 8 | (u32)can_dat->DataBuffer[3]);
            }
        }
    }
    if (can_dat->DLC == 3) // ���ݳ��ȱ�����1,fifoƵ�ʳ����˾Ͳ����տ�����
    {
        if (can_dat->ID == User_Can_ID) // ID����,������Ч
        {
            if (can_dat->DataBuffer[2] & 0x80)
            {
                // LEFT
                camera_dat_l = (int)can_dat->DataBuffer[0] << 8 | (int)can_dat->DataBuffer[1];
                camera_state_l = (u8)can_dat->DataBuffer[2] & 0x7f;
            }
            else
            {
                camera_dat_r = (int)can_dat->DataBuffer[0] << 8 | (int)can_dat->DataBuffer[1];
                camera_state_r = (u8)can_dat->DataBuffer[2] & 0x7f;
            }
        }
    }
}



