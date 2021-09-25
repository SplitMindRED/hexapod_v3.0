#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "splitmind_f401_hal_lib.h"
#include "hal_dynamixel_ax12_a.h"

void setup()
{
   initPeriph();

   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void pushButton()
{
   if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1)
   {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
   }
   else
   {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
   }
}

int main()
{
   setup();

   while (1)
   {
      pushButton();

      // uint8_t buf[20];
      // strcpy((char *)buf, "who: \r\n");
      // HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
      // HAL_Delay(500);
      UART_print(UDBG, -9578);
      HAL_Delay(500);
      UART_printStr(UDBG, "hello");
      HAL_Delay(500);
      UART_printStrLn(UDBG, " hello x2 ");
      HAL_Delay(500);
      UART_printLn(UDBG, -100);
      HAL_Delay(500);
      UART_printDivLn(UDBG, -0.12);
      HAL_Delay(500);
   }

   return 0;
}
