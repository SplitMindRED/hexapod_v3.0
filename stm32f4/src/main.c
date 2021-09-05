#include "splitmind_stm32f401_lib.h"
#include "dynamixel_ax12_a.h"

void init()
{
   //change freq to 84 MHz
   changeCoreFrequency();

   //disable all interruptions
   __disable_irq();

   systemTimeInit();

   // usartInit();

   // usartHalfDuplexInit();

   usartHalfDuplexInit1();

   //enable all interruptions
   __enable_irq();

   //disable uart rx interruption, so i will not catch my own tx bytes
   // USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);
   USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

   GPIO_InitTypeDef  GPIO_InitStructure;

   /* Initialize LED which connected to PA5 */
   // Enable PORT_A Clock
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

   /* Configure the GPIO_LED pin */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   // GPIO_SetBits(GPIOA, GPIO_Pin_5); // Set C13 to High level ("1")
   GPIO_ResetBits(GPIOA, GPIO_Pin_5); // Set C13 to Low level ("0")


/* Initialize Button input PC13 */
// Enable PORTB Clock
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

   /* Configure the GPIO_BUTTON pin */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void testButton()
{
   if (GPIOC->IDR & (1 << 13))
   {
      GPIO_SetBits(GPIOA, GPIO_Pin_5); // Set C13 to High level ("1")
      flag = true;
   }
   else
   {
      GPIO_ResetBits(GPIOA, GPIO_Pin_5); // Set C13 to Low level ("0")

      if (flag == true)
      {
         // sendByteln('b');
         // sendByteln('c');
         // sendByteln(byte1);
         // uint8_t ar[3] = { 'a', 'b', 'c' };
         // sendByteArray(ar, 3);

         // setEndless(1, 1);
         // turn(1, 100);
         // delayms(500);

         flag = false;
      }
   }
}

void testMove(uint16_t pause)
{
   setEndless(2, 1);
   turn(2, 100);
   delayms(pause);
   turn(1, 250);
   turn(2, 250 + 1024);
   delayms(pause);
}

int main(void)
{
   init();

   // pingServo(1);
   // delayms(100);

   // changeID(2);

   // jointMode(2);
   // setEndless(1, 1);

   setEndless(2, 1);
   turn(2, 100);

   double angle_to_hex = 1024.0 / 300.0;
   double start_angle = 150 * angle_to_hex;
   uint32_t pause = 1000;

   while (1)
   {
      // testButton();

      testMove(500);

      // jointMode(2);
      // delayms(100);
      // setAngle(2, 1023);
      // delayms(pause);

      // jointMode(2);
      // delayms(100);
      // setAngle(2, (uint16_t)(start_angle + 90 * angle_to_hex));
      // delayms(pause);

      // setAngle(2, (uint16_t)start_angle);
      // delayms(pause);
      // setAngle(2, (uint16_t)(start_angle - 90 * angle_to_hex));
      // delayms(pause);
      // setAngle(2, 0);
      // delayms(pause);

      // for (uint16_t i = 0; i < 2047; i++)
      // {
      //    turn(1, i);
      //    delayms(10);
      // }

      // for (uint16_t i = 1024; i < 2047; i++)
      // {
      //    turn(1, i);
      //    delayms(10);
      // }

      // delayms(100);
      // pingServo(1);
      // delayms(100);
      // pingServo(2);
      // delayms(100);

      // getActualPosition(1);

      // delayms(500);
      // GPIO_SetBits(GPIOA, GPIO_Pin_5); // Set C13 to High level ("1")
      // delayms(500);
      // GPIO_ResetBits(GPIOA, GPIO_Pin_5); // Set C13 to Low level ("0")

      // delayms(500);

      // if (!setServoAngle(1, 150.0))
      // {
      //    int ab = 0;
      // }

      // for (uint8_t i = 0; i < 253; i++)
      // {
      //    pingServo(i);
      //    id = i;

      //    delayms(10);
      // }

      // GPIO_ResetBits(GPIOA, GPIO_Pin_5); // Set C13 to Low level ("0")
   }

   return 0;
}


