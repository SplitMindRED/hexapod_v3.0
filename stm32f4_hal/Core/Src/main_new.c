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
   wheelMode(1, 1);
   wheelMode(2, 1);

   setVelocity(1, 100);
   setVelocity(2, 100);

   HAL_Delay(pause);
   setVelocity(1, 100 + 1024);
   setVelocity(2, 100 + 1024);
   HAL_Delay(pause);
}

int main()
{
   setup();

   while (1)
   {
      pingServo(1);
      pingServo(2);

      // HAL_Delay(100);
      // pushButton();
      testMove(500);

      // unsigned long t1 = 0, t2 = 0;

      // t1 = HAL_GetTick();
      getPosition(2);
      // t2 = HAL_GetTick();

      // UART_printStr("t: ");
      // UART_printLn(t2 - t1);

      // UART_printLn(HAL_GetTick());
   }

   return 0;
}
