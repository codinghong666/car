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
/* 如果要在程序中使用此代码,请在程序中注明使用了STC的资料及程序            */
/*---------------------------------------------------------------------*/

#ifndef	__STC32G_NVIC_H
#define	__STC32G_NVIC_H

#include	"config.h"



//CAN1中断优先级控制
#define 	CAN1_Priority(n)		do{if(n == 0) PCANH = 0, PCANL = 0; \
																if(n == 1) PCANH = 0, PCANL = 1; \
																if(n == 2) PCANH = 1, PCANL = 0; \
																if(n == 3) PCANH = 1, PCANL = 1; \
															}while(0)

//CAN2中断优先级控制
#define 	CAN2_Priority(n)		do{if(n == 0) PCAN2H = 0, PCAN2L = 0; \
																if(n == 1) PCAN2H = 0, PCAN2L = 1; \
																if(n == 2) PCAN2H = 1, PCAN2L = 0; \
																if(n == 3) PCAN2H = 1, PCAN2L = 1; \
															}while(0)

//LIN中断优先级控制
#define 	LIN_Priority(n)		do{if(n == 0) PLINH = 0, PLINL = 0; \
																if(n == 1) PLINH = 0, PLINL = 1; \
																if(n == 2) PLINH = 1, PLINL = 0; \
																if(n == 3) PLINH = 1, PLINL = 1; \
															}while(0)


//========================================================================
//                           外部函数和变量声明
//========================================================================
u8 NVIC_CAN_Init(u8 Channel, u8 State, u8 Priority);

#endif

