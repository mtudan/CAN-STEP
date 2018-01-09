/*
 ******************************************************************************
 * File Name          : stepper.h
 * Description        : Defines for stepper part of application
 ******************************************************************************
 *
 * Copyright (c) 2018 Matija Tudan 
 * All rights reserved
 *
 ******************************************************************************
 */

#include <stdint.h>
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STEPPER_H
#define __STEPPER_H

/* Private defines -----------------------------------------------------------*/
#define STEPPER_WAVING	200
#define STEPS_NUM				400
#define ANGLE_PER_STEP  0.9		// (360 / STEPS_NUM)

typedef struct
{
	uint16_t StepsNumber;
	uint8_t SwitchRotationDirection;
	uint16_t RotationAngle;
	uint8_t RotationSpeed;
	uint8_t ContinuousRotation;
	uint8_t StartStopContinuousRotation;
} StepperConfiguration;

#endif /* __STEPPER_H */
