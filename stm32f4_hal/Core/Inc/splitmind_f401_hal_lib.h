#ifndef SPLITMIND_F401_HAL_LIB
#define SPLITMIND_F401_HAL_LIB

#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "math.h"
#include "main.h"

//UART for debug
#define UDBG &huart2

extern volatile unsigned long system_time;

typedef struct SoftTimer_ms
{
   unsigned long delay;
   unsigned long start_time;
}SoftTimer_ms;

bool checkTimer(SoftTimer_ms *timer);

/*
  ******************************************************************************
  * UART print stuff
  ******************************************************************************
*/
void UART_sendByte(UART_HandleTypeDef *huart, char byte);
void UART_print(UART_HandleTypeDef *huart, long data);
void UART_printStr(UART_HandleTypeDef *huart, char *string);
void UART_printDiv(UART_HandleTypeDef *huart, double data);
void UART_printLn(UART_HandleTypeDef *huart, long data);
void UART_printStrLn(UART_HandleTypeDef *huart, char *string);
void UART_printDivLn(UART_HandleTypeDef *huart, double data);
/*
  ******************************************************************************
*/

#endif /* __SPLITMIND_F401_HAL_LIB */
