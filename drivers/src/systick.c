/*
 * systick.c
 *
 *  Created on: Aug 16, 2024
 *      Author: Karan Patel
 */

#include "systick.h"



void SystickDelayMs(int delay){
	//Reload with number of clocks per millisecond
  SysTick->LOAD = 16-1;

  // Clear systick current value register.
  SysTick->VAL =0;

  // Enable systick and select internal clk source.
  SysTick->CTRL = (1<<0) | (1<<2);

  for(int i=0;i<delay;i++){
	  //wait until the CNTFLAG is set
	  while((SysTick->CTRL & (1<<16))==0){

	  }
  }
  SysTick->CTRL =0;
}

void Systick_Interrupt_1_S(){
	//Reload with number of clocks
  SysTick->LOAD = 16000000 - 1  ;
  // Clear systick current value register.
  SysTick->VAL =0;
  // Enable systick and select internal clk source.
  SysTick->CTRL = (1<<0) | (1<<2);
  // enable the interrupt tickcnt bit
  SysTick->CTRL |= (1<<1);
}
