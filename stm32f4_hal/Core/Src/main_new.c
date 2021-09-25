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

void testMove(uint16_t pause)
{
   setEndless(UART1, 2, 1);
   turn(UART1, 2, 100);
   HAL_Delay(pause);
   // turn(UART1, 2, 100 + 1024);
   HAL_Delay(pause);
}

int main()
{
   setup();

   while (1)
   {
      pingServo(UART1, 2);

      // uint8_t response1[20];

      // for (uint8_t i = 0; i < 20; i++)
      // {
      //    response1[i] = 0;
      // }

      // HAL_UART_Receive(UART2, response1, 3, HAL_MAX_DELAY);


      HAL_Delay(100);
      // pushButton();
      testMove(500);

      getActualPosition(UART1, 2);
      // UART_printLn(HAL_GetTick());
   }

   return 0;
}
