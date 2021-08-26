#include "splitmind_stm32f401_lib.h"
#include "dynamixel_ax12_a.h"

void clearServoReceiveBuffer(void)
{
   receiveBufferStart = receiveBufferEnd;
}

uint8_t getServoBytesAvailable(void)
{
   volatile uint8_t* start = receiveBufferStart;
   volatile uint8_t* end = receiveBufferEnd;

   if (end >= start)
   {
      return (uint8_t)(end - start);
   }
   else
   {
      return (uint8_t)(REC_BUFFER_LEN - (start - end));
   }
}

uint8_t getServoByte(void)
{
   receiveBufferStart++;
   if (receiveBufferStart >= receiveBuffer + REC_BUFFER_LEN)
   {
      receiveBufferStart = receiveBuffer;
   }

   return *receiveBufferStart;
}

void init()
{
   //change freq to 84 MHz
   changeCoreFrequency();

   //disable all interruptions
   __disable_irq();

   systemTimeInit();

   // usartInit();
   usartHalfDuplexInit();

   //enable all interruptions
   __enable_irq();

   //disable uart rx interruption, so i will not catch my own tx bytes
   USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);

   GPIO_InitTypeDef  GPIO_InitStructure;

   /* Initialize LED which connected to PA5 */
   // Enable PORT_A Clock
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

   /* Configure the GPIO_LED pin */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   GPIO_SetBits(GPIOA, GPIO_Pin_5); // Set C13 to High level ("1")

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

int main(void)
{
   init();

   while (1)
   {
      testButton();

      setEndless(1, 1);
      turn(1, 100);
      delayms(10);

      pingServo(1);
      delayms(10);
      int b = 0;

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


