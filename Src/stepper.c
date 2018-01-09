/*
 ******************************************************************************
 * File Name          : stepper.c
 * Description        : Code for stepper part of application
 ******************************************************************************
 *
 * Copyright (c) 2018 Matija Tudan 
 * All rights reserved
 *
 ******************************************************************************
 */

#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "can.h"
#include "stepper.h"

/* Structure to hold Stepper Configuration */
StepperConfiguration StepperConfig;

/* Extern functions */
extern void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
extern void CAN_TX(uint32_t CAN_ID, uint8_t CAN_DLC, uint8_t CAN_data[8]);

/* Handle to Stepper timer */
TimerHandle_t xStepperTimer;

/* Extern and auxiliary variables */
extern bool firstBoot;
extern uint8_t can_tx_msg[8];
unsigned int step_num_cnt = 0;
uint8_t speed_ctrl_cnt = 0;

/* Perform waving with stepper on first boot */
void firstBootStepperTimerCallback()
{
	step_num_cnt++;
	HAL_GPIO_TogglePin(STEP_GPIO_Port, STEP_Pin);
	
	if (step_num_cnt == STEPPER_WAVING)
	{
		HAL_GPIO_TogglePin(DIR_GPIO_Port, DIR_Pin);
	}
	else if (step_num_cnt == (2 * STEPPER_WAVING))
	{
		firstBoot = false;
		step_num_cnt = 0;
		xTimerStop(xStepperTimer, 0);
		
		can_tx_msg[0] = CAN_FIN_F0;
		can_tx_msg[1] = CAN_FIN_0D;
		CAN_TX(CAN_TX_ID, 2, can_tx_msg);
		
		// Going full speed after first boot on every incoming CAN msg (0x10 and 0x20)
		StepperConfig.RotationSpeed = 1;
	}
}

/* Regular stepper behavior after first boot */
void regularStepperTimerCallback()
{
	if (StepperConfig.RotationSpeed != 0)
	{
		// Start rotating if speed is not equal to zero
		speed_ctrl_cnt++;
		
		if (speed_ctrl_cnt == StepperConfig.RotationSpeed)
		{
			speed_ctrl_cnt = 0;
			
			HAL_GPIO_TogglePin(STEP_GPIO_Port, STEP_Pin);
			
			// If continuous rotation is not being performed,
			// count steps and eventually stop rotating
			if (!StepperConfig.ContinuousRotation)
			{
				step_num_cnt++;
				
				if (step_num_cnt == StepperConfig.StepsNumber)
				{
					step_num_cnt = 0;
					xTimerStop(xStepperTimer, 0);
					
					can_tx_msg[1] = CAN_FIN_F0;
					can_tx_msg[2] = CAN_FIN_0D;
					CAN_TX(CAN_TX_ID, 3, can_tx_msg);
				}
			}
		}
	}
	else
	{
		// Don't rotate if speed is set to zero
		xTimerStop(xStepperTimer, 0);
	}
}

/* Callback function of Stepper timer */
void vStepperTimerCallback(TimerHandle_t xTimer)
{
	if (firstBoot)
	{
		firstBootStepperTimerCallback();
	}
	else
	{
		regularStepperTimerCallback();
	}
}

void startStepperTimer(void)
{
	xStepperTimer = xTimerCreate(/* Just a text name, not used by the RTOS kernel. */
																"Stepper Timer",
																/* The timer period in ticks, must be greater than 0. */
																1,
																/* The timers will auto-reload themselves when they expire. */
																pdTRUE,
																/* The ID is used to store a count of the number of times the timer has expired, which is initialised to 0. */
																(void*) 0,
																/* Each timer calls the same callback when it expires. */
																vStepperTimerCallback
															);
	
	if (xStepperTimer == NULL)
	{
		/* The timer was not created. */
	}
	else
	{
		/* Start the timer.  No block time is specified, and
		even if one was it would be ignored because the RTOS
		scheduler has not yet been started. */
		
		if (xTimerStart(xStepperTimer, 0) != pdPASS)
		{
			/* The timer could not be set into the Active state. */
		}
	}
}
