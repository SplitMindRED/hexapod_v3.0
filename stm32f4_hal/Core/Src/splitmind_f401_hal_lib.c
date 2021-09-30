#include "splitmind_f401_hal_lib.h"

volatile unsigned long system_time = 0;

void turnLed(bool status)
{
//   if (status == 1)
//   {
//      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//   }
//   else
//   {
//      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//   }
}


/*
  ******************************************************************************
  * UART print stuff
  ******************************************************************************
*/

#ifdef UART_DEBUG

void UART_printNumber(unsigned long number)
{
   char string[11];
   char *pointer = &string[sizeof(string) - 1];

   *pointer = '\0';
   pointer--;

   if (number == 0)
   {
      *(pointer--) = '0';
   }
   else
   {
      while (number)
      {
         char symbol = number % 10;
         number = number / 10;

         *(pointer--) = symbol + '0';
      }
   }

   HAL_UART_Transmit(UDBG, (uint8_t *)(pointer + 1), strlen((char *)(pointer + 1)), HAL_MAX_DELAY);

}

void UART_sendByte(char byte)
{
   HAL_UART_Transmit(UDBG, (uint8_t *)(&byte), 1, HAL_MAX_DELAY);
}

void UART_print(long data)
{
   if (data < 0)
   {
      UART_sendByte('-');
      data = -data;
   }

   UART_printNumber(data);
}

void UART_printStr(char *string)
{
   HAL_UART_Transmit(UDBG, (uint8_t *)string, strlen(string), HAL_MAX_DELAY);
}

void UART_printDiv(double data)
{
   if (isnan(data))
   {
      UART_printStr("nan");
      return;
   }
   else if (isinf(data))
   {
      UART_printStr("inf");
      return;
   }
   else if (data > 4294967040.0)
   {
      UART_printStr("ovf");
      return;
   }
   else if (data < -4294967040.0)
   {
      UART_printStr("-ovf");
      return;
   }

   if (data < 0.0)
   {
      UART_sendByte('-');
      data = -data;
   }

   long rounding = 0;
   long number_left = data;
   uint8_t number_right = 0;

   rounding = data * 1000;
   uint8_t tmp = rounding % 10;
   rounding = rounding / 10;

   if (tmp >= 5)
   {
      number_right = (long)(rounding + 1) % 100;
   }
   else
   {
      number_right = (long)rounding % 100;
   }

   UART_print(number_left);
   UART_sendByte('.');
   UART_print(number_right);
}

void UART_printLn(long data)
{
   UART_print(data);

   UART_sendByte('\n');
}

void UART_printStrLn(char *string)
{
   UART_printStr(string);

   UART_sendByte('\n');
}

void UART_printDivLn(double data)
{
   UART_printDiv(data);

   UART_sendByte('\n');
}

#endif
/*
  ******************************************************************************
*/
