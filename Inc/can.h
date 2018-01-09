/*
 ******************************************************************************
 * File Name          : can.h
 * Description        : Defines for CAN part of application
 ******************************************************************************
 *
 * Copyright (c) 2018 Matija Tudan 
 * All rights reserved
 *
 ******************************************************************************
 */
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H
#define __CAN_H

/* Private defines -----------------------------------------------------------*/
#define CAN_RX_ID		0x400
#define CAN_TX_ID		0x500
#define CAN_ACK_C0	0xC0
#define CAN_ACK_01	0x01
#define CAN_FIN_F0	0xF0
#define CAN_FIN_0D	0x0D

#define	STEP_NUM_DIR	0x10 
#define	ROT_ANG_DIR 	0x20 
#define	ROT_SPEED 		0x30 
#define	ROT_DIR_CONT 	0x40
	
#endif /* __CAN_H */
