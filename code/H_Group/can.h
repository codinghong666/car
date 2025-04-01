#ifndef __CAN_H_
#define __CAN_H_

#include "./H_Group/config.h"

#define	STANDARD_FRAME   0     //֡��ʽ����׼֡
#define	EXTENDED_FRAME   1     //֡��ʽ����չ֡

#define CAN1 0
#define CAN2 1

typedef struct
{
	u8	DLC:4;          //���ݳ���, bit0~bit3
	u8	:2;             //������, bit4~bit5
	u8	RTR:1;          //֡����, bit6
	u8	FF:1;           //֡��ʽ, bit7
	u32	ID;             //CAN ID
	u8	DataBuffer[8];  //���ݻ���
}CAN_DataDef;

extern bit B_Can1Send;
extern bit B_Can1Read;

extern CAN_DataDef CAN1_Tx;
extern CAN_DataDef CAN1_Rx[8];

extern long read_left_postion, read_right_postion;

void Can_Init(void);
u8 CanReadReg(u8 addr);
u8 CanReadMsg(CAN_DataDef *CAN);
void CanSendMsg(CAN_DataDef *CAN);
void Can_Dat_Handle(CAN_DataDef *can_dat);
extern int camera_dat_l, camera_dat_r;
extern u8 camera_state_l, camera_state_r;
extern bit can_send_dat_flag;
//�û�CAN����Э�鶨��
//���ݶ˶�Ӧ
#define Dat_Ctrl 0

//bitλ��Ӧ
#define Ctrl_LED 0x80
#define Ctrl_Auto 0x40
#define Ctrl_Left 0x20
#define Ctrl_Right 0x10
#define Ctrl_Normal 0x08

#endif 