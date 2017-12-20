/*
 ******************************************************************************
 * File Name          : stepper.c
 * Description        : Code for stepper part of application
 ******************************************************************************
 *
 * Copyright (c) 2017 Matija Tudan 
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other 
 *    contributors to this software may be used to endorse or promote products 
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this 
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under 
 *    this license is void and will automatically terminate your rights under 
 *    this license. 
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

extern void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/* An array to hold handles to the created timers. */
TimerHandle_t xTimer;

/* Define a callback function that will be used by timer instances.
   The callback function toggles LED when the associated timer expires. */
void vTimerCallback(TimerHandle_t xTimer)
{
	HAL_GPIO_TogglePin(STEP_GPIO_Port, STEP_Pin);
}

void startStepperTimer(void)
{
	xTimer = xTimerCreate(/* Just a text name, not used by the RTOS kernel. */
												"Stepper Timer",
												/* The timer period in ticks, must be greater than 0. */
												1000,
												/* The timers will auto-reload themselves when they expire. */
												pdTRUE,
												/* The ID is used to store a count of the number of times the timer has expired, which is initialised to 0. */
												(void*) 0,
												/* Each timer calls the same callback when it expires. */
												vTimerCallback
											 );

	if (xTimer == NULL)
	{
		/* The timer was not created. */
	}
	else
	{
		/* Start the timer.  No block time is specified, and
		even if one was it would be ignored because the RTOS
		scheduler has not yet been started. */
		if (xTimerStart(xTimer, 0) != pdPASS)
		{
			/* The timer could not be set into the Active state. */
		}
	}
}

/* Task to be created. */
void vTaskCode(void * pvParameters)
{
    /* The parameter value is expected to be 1 as 1 is passed in the
    pvParameters value in the call to xTaskCreate() below. 
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 ); */

    for( ;; )
    {
      /* Task code goes here. */
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			vTaskDelay(1000);
    }
}

/* Function that creates a task. */
void createStepperTask(void)
{
	BaseType_t xReturned;
	TaskHandle_t xHandle = NULL;
	
  /* Create the task, storing the handle. */
  xReturned = xTaskCreate(
                  vTaskCode,       	/* Function that implements the task. */
                  "NAME",          	/* Text name for the task. */
                  128,      				/* Stack size in words, not bytes. */
                  (void *) 1,    		/* Parameter passed into the task. */
                  tskIDLE_PRIORITY,	/* Priority at which the task is created. */
                  &xHandle					/* Used to pass out the created task's handle. */
							);
	
  if(xReturned == pdPASS)
  {
		/* The task was created.  Use the task's handle to delete the task. */
    //vTaskDelete(xHandle);
  }
}
