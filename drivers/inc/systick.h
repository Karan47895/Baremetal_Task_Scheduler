/*
 * systick.h
 *
 *  Created on: Aug 16, 2024
 *      Author: Karan Patel
 */

#ifndef INC_SYSTICK_H_
#define INC_SYSTICK_H_


#include "stm32f407xx.h"

void SystickDelayMs(int delay);
void Systick_Interrupt_1_S();


#endif /* INC_SYSTICK_H_ */
