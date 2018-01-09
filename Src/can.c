/*
 ******************************************************************************
 * File Name          : can.c
 * Description        : Code for CAN part of application
 ******************************************************************************
 *
 * Copyright (c) 2018 Matija Tudan 
 * All rights reserved
 *
 ******************************************************************************
 */

#include <string.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "can.h"
#include "stepper.h"

/* CAN global variables */
QueueHandle_t xCanRxQueue;
extern CAN_HandleTypeDef hcan;
extern CanTxMsgTypeDef CAN_TX_Msg;
CanRxMsgTypeDef CAN_RX_QueueMsg;
uint8_t can_tx_msg[8];

/* Extern Stepper Configuration structure */
extern StepperConfiguration StepperConfig;

/* Stepper timer extern variable */
extern TimerHandle_t xStepperTimer;

/* Toggle GPIO pin function from HAL */
extern void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/* Transmit CAN message */
void CAN_TX(uint32_t CAN_ID, uint8_t CAN_DLC, uint8_t CAN_data[8])
{
	CAN_TX_Msg.StdId = CAN_ID;
	CAN_TX_Msg.DLC = CAN_DLC;
	memcpy(&CAN_TX_Msg.Data, &CAN_data[0], sizeof(CAN_TX_Msg.Data));
	
	HAL_CAN_Transmit_IT(&hcan);
}

/* Determine what to do with the received CAN message */
void checkCanMsg(CanRxMsgTypeDef CAN_RX_Msg)
{
	if (CAN_RX_Msg.StdId == CAN_RX_ID)
	{
		switch (CAN_RX_Msg.Data[0])
    {
    	case STEP_NUM_DIR:
			{
				can_tx_msg[0] = STEP_NUM_DIR;
				can_tx_msg[1] = CAN_ACK_C0;
				can_tx_msg[2] = CAN_ACK_01;
				CAN_TX(CAN_TX_ID, 3, can_tx_msg);
			
				StepperConfig.StepsNumber = 2 * (CAN_RX_Msg.Data[1] + CAN_RX_Msg.Data[2]);
				StepperConfig.SwitchRotationDirection = CAN_RX_Msg.Data[3];
				if (StepperConfig.SwitchRotationDirection == 1)
					HAL_GPIO_TogglePin(DIR_GPIO_Port, DIR_Pin);
				xTimerStart(xStepperTimer, 0);
    		break;
			}
				
    	case ROT_ANG_DIR:
			{
				can_tx_msg[0] = ROT_ANG_DIR;
				can_tx_msg[1] = CAN_ACK_C0;
				can_tx_msg[2] = CAN_ACK_01;
				CAN_TX(CAN_TX_ID, 3, can_tx_msg);
				
				StepperConfig.RotationAngle = CAN_RX_Msg.Data[1] + CAN_RX_Msg.Data[2];
				StepperConfig.StepsNumber = 2 * (StepperConfig.RotationAngle / ANGLE_PER_STEP);
				StepperConfig.SwitchRotationDirection = CAN_RX_Msg.Data[3];
				if (StepperConfig.SwitchRotationDirection == 1)
					HAL_GPIO_TogglePin(DIR_GPIO_Port, DIR_Pin);
				xTimerStart(xStepperTimer, 0);
    		break;
			}
			
			case ROT_SPEED:
			{
				can_tx_msg[0] = ROT_SPEED;
				can_tx_msg[1] = CAN_ACK_C0;
				can_tx_msg[2] = CAN_ACK_01;
				CAN_TX(CAN_TX_ID, 3, can_tx_msg);
			
				StepperConfig.RotationSpeed = (256 - CAN_RX_Msg.Data[1]);
    		break;
			}
			
			case ROT_DIR_CONT:
			{
				can_tx_msg[0] = ROT_DIR_CONT;
				can_tx_msg[1] = CAN_ACK_C0;
				can_tx_msg[2] = CAN_ACK_01;
				CAN_TX(CAN_TX_ID, 3, can_tx_msg);
			
				StepperConfig.ContinuousRotation = true;
				
				StepperConfig.SwitchRotationDirection = CAN_RX_Msg.Data[1];
				if (StepperConfig.SwitchRotationDirection == 1)
					HAL_GPIO_TogglePin(DIR_GPIO_Port, DIR_Pin);
				
				StepperConfig.StartStopContinuousRotation = CAN_RX_Msg.Data[2];
				if (StepperConfig.StartStopContinuousRotation == 1)
				{
					xTimerStart(xStepperTimer, 0);
				}
				else if (StepperConfig.StartStopContinuousRotation == 0)
				{
					can_tx_msg[1] = CAN_FIN_F0;
					can_tx_msg[2] = CAN_FIN_0D;
					CAN_TX(CAN_TX_ID, 3, can_tx_msg);
					
					StepperConfig.ContinuousRotation = false;
					xTimerStop(xStepperTimer, 0);
				}
    		break;
			}
    }
	}
	else
	{
		// Unrecognized CAN msg, nothing to do
	}
}

/* Create CAN RX Queue */
void createCanRxQueue(void)
{
  /* Create a queue capable of containing 1 CanRxMsgTypeDef structure */
  xCanRxQueue = xQueueCreate(1, sizeof(CanRxMsgTypeDef));

  if (xCanRxQueue == NULL)
  {
		/* Queue was not created and must not be used */
  }
}

/* CAN task used to receive CAN msg from queue and check the received msg */
void vCanTaskCode(void * pvParameters)
{
	for(;;)
	{
		if (xQueueReceive(xCanRxQueue, &CAN_RX_QueueMsg, 0))
		{
			checkCanMsg(CAN_RX_QueueMsg);
		}
	}
}

/* Function that creates CAN task */
void createCanTask(void)
{
	BaseType_t xReturned;
	TaskHandle_t xHandle = NULL;
	
  /* Create the task, storing the handle */
  xReturned = xTaskCreate(
                  vCanTaskCode,			/* Function that implements the task. */
                  "CAN Task",      	/* Text name for the task. */
                  128,      				/* Stack size in words, not bytes. */
                  (void *) 1,    		/* Parameter passed into the task. */
                  tskIDLE_PRIORITY,	/* Priority at which the task is created. */
                  &xHandle					/* Used to pass out the created task's handle. */
							);
	
  if (xReturned == pdPASS)
  {
		/* The task was created */
  }
}
