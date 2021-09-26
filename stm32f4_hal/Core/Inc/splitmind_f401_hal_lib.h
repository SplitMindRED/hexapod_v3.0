/***********************************************
*	SplitMind Library for STM32F401RE using HAL
*	Version 0.2
*
*  Functions for low level initialization and control
*
*	WRITING BITs
*	a |= 1 << 7;					//set 1 in 7th bit
*	a &= ~(1 << 3);				//set 0 in 3th bit
*	a ^= 1 << 5;					//inversion of 5th bit
*	a |= 1 << 7 | 1 << 8		   //sets 1 to 7th and 8th bits
*
*	READING BITs
*	if( a & (1<<7) )				//if 7th bit in "a" var equals 1 -> true
*  a & ( 1 << 7 | 1 << 8 )	   //checks 7th and 8th bits
*
************************************************/

#ifndef SPLITMIND_F401_HAL_LIB
#define SPLITMIND_F401_HAL_LIB

#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "math.h"
#include "main.h"
#include "usart.h"
#include "gpio.h"

#define UART_DEBUG

#define MAX_DELAY 15
#define OK        1
#define ERROR     -1

//UART for debug
#ifdef UART_DEBUG
#define UDBG &huart2
#endif

#define UART1 &huart1
#define UART2 &huart2
#define UART6 &huart6

extern volatile unsigned long system_time;

typedef struct SoftTimer_ms
{
   unsigned long delay;
   unsigned long start_time;
}SoftTimer_ms;

bool checkTimer(SoftTimer_ms *timer);

void turnLed(bool status);

/*
  ******************************************************************************
  * UART print stuff
  ******************************************************************************
*/
#ifdef UART_DEBUG
void UART_sendByte(char byte);
void UART_print(long data);
void UART_printStr(char *string);
void UART_printDiv(double data);
void UART_printLn(long data);
void UART_printStrLn(char *string);
void UART_printDivLn(double data);
#endif
/*
  ******************************************************************************
*/

#endif /* __SPLITMIND_F401_HAL_LIB */
