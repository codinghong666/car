/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC 1T Series MCU Demo Programme -------------------------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966 ------------------------*/
/* --- Web: www.STCAI.com ---------------------------------------------*/
/* --- Web: www.STCMCUDATA.com  ---------------------------------------*/
/* --- BBS: www.STCAIMCU.com  -----------------------------------------*/
/* --- QQ:  800003751 -------------------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ����STC�����ϼ�����            */
/*---------------------------------------------------------------------*/

#include "STC32G_NVIC.h"

//========================================================================
// ����: NVIC_CAN_Init
// ����: CANǶ�������жϿ�������ʼ��.
// ����: Channel:  ͨ��, CAN1/CAN2.
// ����: State:    �ж�ʹ��״̬, ENABLE/DISABLE.
// ����: Priority: �ж����ȼ�, Priority_0,Priority_1,Priority_2,Priority_3.
// ����: ִ�н�� SUCCESS/FAIL.
// �汾: V1.0, 2023-03-27
//========================================================================
#ifndef CAN1
#define	CAN1	0
#endif
#ifndef CAN2
#define	CAN2	1
#endif
u8 NVIC_CAN_Init(u8 Channel, u8 State, u8 Priority)
{
	if(Channel > CAN2) return FAIL;
	if(Priority > Priority_3) return FAIL;
	switch(Channel)
	{
		case CAN1:
			if(State == ENABLE)
				CANIE = 1;		//bit7 1:Enable Interrupt
			else
				CANIE = 0;		//bit7 0:Disable Interrupt
			CAN1_Priority(Priority);
		break;

		case CAN2:
			if(State == ENABLE)
				CAN2IE = 1;		//bit7 1:Enable Interrupt
			else
				CAN2IE = 0;		//bit7 0:Disable Interrupt
			CAN2_Priority(Priority);
		break;

		default:
			return FAIL;
		break;
	}
	return SUCCESS;
}

