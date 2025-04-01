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
    CanWriteReg(MR, 0x04);   // 使能Reset模式
    CanWriteReg(BTR0, 0xc3); // SJW(3), BRP(3)
    CanWriteReg(BTR1, 0xc6); // SAM(1), TSG2(4), TSG1(6)
    CanWriteReg(MR, 0x00);   // 退出Reset模式
}

void CanReadFifo(CAN_DataDef *CAN)
{
    u8 i;
    u8 pdat[5];
    u8 RX_Index = 0;

    pdat[0] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3)));

    if (pdat[0] & 0x80) // 判断是标准帧还是扩展帧
    {
        pdat[1] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3))); // 扩展帧ID占4个字节
        pdat[2] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3)));
        pdat[3] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3)));
        pdat[4] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3)));
        CAN->ID = (((u32)pdat[1] << 24) + ((u32)pdat[2] << 16) + ((u32)pdat[3] << 8) + pdat[4]) >> 3;
    }
    else
    {
        pdat[1] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3))); // 标准帧ID占2个字节
        pdat[2] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3)));
        CAN->ID = ((pdat[1] << 8) + pdat[2]) >> 5;
    }

    CAN->FF = pdat[0] >> 7;  // 帧格式
    CAN->RTR = pdat[0] >> 6; // 帧类型
    CAN->DLC = pdat[0];      // 数据长度

    for (i = 0; ((i < CAN->DLC) && (i < 8)); i++) // 读取数据长度为len，最多不超过8
    {
        CAN->DataBuffer[i] = CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3))); // 读取有效数据
    }
    while (RX_Index & 3) // 判断已读数据长度是否4的整数倍
    {
        CanReadReg((u8)(RX_BUF0 + (RX_Index++ & 3))); // 读取填充数据，一帧数据占据4的整数倍缓冲区空间，不足补0
    }
}

u8 CanReadMsg(CAN_DataDef *CAN)
{
    u8 i;
    u8 n = 0;
    do
    {
        CanReadFifo(&CAN[n++]); // 读取接收缓冲区数据
        i = CanReadReg(SR);
    } while (i & 0x80); // 判断接收缓冲区里是否还有数据，有的话继续读取

    return n; // 返回帧个数
}

void CanSendMsg(CAN_DataDef *CAN)
{
    u32 CanID;
    u8 RX_Index, i;

    if (CANSEL == CAN1) // 判断是否CAN1
    {
        while (B_Can1Send)
            ; // 等待CAN1上次发送完成
    }

    if (CAN->FF) // 判断是否扩展帧
    {
        CanID = CAN->ID << 3;
        CanWriteReg(TX_BUF0, CAN->DLC | ((u8)CAN->RTR << 6) | 0x80); // bit7: 标准帧(0)/扩展帧(1), bit6: 数据帧(0)/远程帧(1), bit3~bit0: 数据长度(DLC)
        CanWriteReg(TX_BUF1, (u8)(CanID >> 24));
        CanWriteReg(TX_BUF2, (u8)(CanID >> 16));
        CanWriteReg(TX_BUF3, (u8)(CanID >> 8));
        CanWriteReg(TX_BUF0, (u8)CanID);

        RX_Index = 1;
        for (i = 0; ((i < CAN->DLC) && (i < 8)); i++) // 数据长度为DLC，最多不超过8
        {
            CanWriteReg((u8)(TX_BUF0 + (RX_Index++ & 3)), CAN->DataBuffer[i]); // 写入有效数据
        }
        while (RX_Index & 3) // 判断已读数据长度是否4的整数倍
        {
            CanWriteReg((u8)(TX_BUF0 + (RX_Index++ & 3)), 0x00); // 写入填充数据，一帧数据占据4的整数倍缓冲区空间，不足补0
        }
    }
    else // 发送标准帧
    {
        CanID = (u16)(CAN->ID << 5);
        CanWriteReg(TX_BUF0, CAN->DLC | ((u8)CAN->RTR << 6)); // bit7: 标准帧(0)/扩展帧(1), bit6: 数据帧(0)/远程帧(1), bit3~bit0: 数据长度(DLC)
        CanWriteReg(TX_BUF1, (u8)(CanID >> 8));
        CanWriteReg(TX_BUF2, (u8)CanID);

        RX_Index = 3;
        for (i = 0; ((i < CAN->DLC) && (i < 8)); i++) // 数据长度为DLC，最多不超过8
        {
            CanWriteReg((u8)(TX_BUF0 + (RX_Index++ & 3)), CAN->DataBuffer[i]); // 写入有效数据
        }
        while (RX_Index & 3) // 判断已读数据长度是否4的整数倍
        {
            CanWriteReg((u8)(TX_BUF0 + (RX_Index++ & 3)), 0x00); // 写入填充数据，一帧数据占据4的整数倍缓冲区空间，不足补0
        }
    }
    CanWriteReg(CMR, 0x04); // 发起一次帧传输

    if (CANSEL == CAN1) // 判断是否CAN1
    {
        B_Can1Send = 1; // 设置CAN1发送忙标志
    }
}

void PortSwitch(void)
{
    P_SW1 &= ~0x30; // CAN1: CANRX(P0.0), CANTX(P0.1)
}

void Can_Init(void)
{
    CANEN = 1;     // 打开CAN1模块
    CANSEL = CAN1; // 选择CAN1模块
    PortSwitch();
    CanSetBaudrate();
    CanWriteReg(MR, 0x05);                     // 使能Reset模式，打开双滤波器功能
    CanWriteReg(ACR0, (u8)(User_Can_ID >> 3)); // 总线验收代码寄存器,过滤设备ID和广播ID
    CanWriteReg(ACR1, (u8)(User_Can_ID << 5));
    CanWriteReg(ACR2, 0x00); // 广播地址默认为0，不需要更改
    CanWriteReg(ACR3, 0x00);

    CanWriteReg(AMR0, 0x00); // 总线验收屏蔽寄存器，数据位不校验，RTR不校验
    CanWriteReg(AMR1, 0x1f);
    CanWriteReg(AMR2, 0x00);
    CanWriteReg(AMR3, 0x1f);

    CanWriteReg(ISR, 0xff); // 清中断标志
    CanWriteReg(IMR, 0xff); // 中断寄存器设置,响应所有中断
    CanWriteReg(MR, 0x00);  // 退出 Reset Mode
    CANICR |= 0x02;         // CAN1中断使能
}

void CANBUS1_Interrupt(void) interrupt CAN1_VECTOR
{
    u8 isr;
    u8 store;
    u8 arTemp;

    arTemp = CANAR; // 先CANAR现场保存，避免主循环里写完 CANAR 后产生中断，在中断里修改了 CANAR 内容
    store = AUXR2;  // 后AUXR2现场保存

    AUXR2 &= ~0x08; // 选择CAN1模块
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
        CANDR &= ~0x04; // 清除 Reset Mode, 从BUS-OFF状态退出

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

    AUXR2 = store;  // 先AUXR2现场恢复
    CANAR = arTemp; // 后CANAR现场恢复
}

int camera_dat_l, camera_dat_r;
u8 camera_state_l, camera_state_r;

long read_left_postion, read_right_postion;

void Can_Dat_Handle(CAN_DataDef *can_dat)
{
    if (can_dat->DLC == 5) // 判定为返回的轮边数据
    {
        if (can_dat->ID == User_Can_ID)
        {
            if (can_dat->DataBuffer[4] == 3) // 左轮
            {
                read_left_postion = (long)((u32)can_dat->DataBuffer[0] << 24 | (u32)can_dat->DataBuffer[1] << 16 | (u32)can_dat->DataBuffer[2] << 8 | (u32)can_dat->DataBuffer[3]);
            }
            if (can_dat->DataBuffer[4] == 2) // 右轮
            {
                read_right_postion = (long)((u32)can_dat->DataBuffer[0] << 24 | (u32)can_dat->DataBuffer[1] << 16 | (u32)can_dat->DataBuffer[2] << 8 | (u32)can_dat->DataBuffer[3]);
            }
        }
    }
    if (can_dat->DLC == 3) // 数据长度必须是1,fifo频率超出了就不接收控制了
    {
        if (can_dat->ID == User_Can_ID) // ID过滤,立刻生效
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



