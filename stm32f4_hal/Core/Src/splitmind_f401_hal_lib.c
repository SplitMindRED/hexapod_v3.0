#include "splitmind_f401_hal_lib.h"

volatile unsigned long system_time = 0;

/*
  ******************************************************************************
  * UART print stuff
  ******************************************************************************
*/

void UART_printNumber(UART_HandleTypeDef *huart, unsigned long number)
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

   HAL_UART_Transmit(huart, (uint8_t *)(pointer + 1), strlen((char *)(pointer + 1)), HAL_MAX_DELAY);
}

void UART_sendByte(UART_HandleTypeDef *huart, char byte)
{
   HAL_UART_Transmit(huart, (uint8_t *)(&byte), 1, HAL_MAX_DELAY);
}

void UART_print(UART_HandleTypeDef *huart, long data)
{
   if (data < 0)
   {
      UART_sendByte(huart, '-');
      data = -data;
   }

   UART_printNumber(huart, data);
}

void UART_printStr(UART_HandleTypeDef *huart, char *string)
{
   HAL_UART_Transmit(huart, (uint8_t *)string, strlen(string), HAL_MAX_DELAY);
}

void UART_printDiv(UART_HandleTypeDef *huart, double data)
{
   if (isnan(data))
   {
      UART_printStr(huart, "nan");
      return;
   }
   else if (isinf(data))
   {
      UART_printStr(huart, "inf");
      return;
   }
   else if (data > 4294967040.0)
   {
      UART_printStr(huart, "ovf");
      return;
   }
   else if (data < -4294967040.0)
   {
      UART_printStr(huart, "-ovf");
      return;
   }

   if (data < 0.0)
   {
      UART_sendByte(huart, '-');
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

   UART_print(huart, number_left);
   UART_sendByte(huart, '.');
   UART_print(huart, number_right);
}

void UART_printLn(UART_HandleTypeDef *huart, long data)
{
   UART_print(huart, data);

   UART_sendByte(huart, '\n');
}

void UART_printStrLn(UART_HandleTypeDef *huart, char *string)
{
   UART_printStr(huart, string);

   UART_sendByte(huart, '\n');
}

void UART_printDivLn(UART_HandleTypeDef *huart, double data)
{
   UART_printDiv(huart, data);

   UART_sendByte(huart, '\n');
}
/*
  ******************************************************************************
*/
