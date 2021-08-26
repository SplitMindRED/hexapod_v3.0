#include "splitmind_stm32f401_lib.h"
#include "dynamixel_ax12_a.h"

#define USART_TX 2
#define USART_RX 3

unsigned char Checksum = 0;
unsigned long delta = 0;

uint8_t servoErrorCode = 0;
uint8_t id = 0;
uint8_t byte1 = 0;
bool flag = false;

uint8_t arr[20];

typedef struct ServoResponse
{
   uint8_t id;
   uint8_t length;
   uint8_t error;
   uint8_t params[SERVO_MAX_PARAMS];
   uint8_t checksum;
} ServoResponse;

ServoResponse response;

volatile uint8_t receiveBuffer[REC_BUFFER_LEN];
volatile uint8_t* volatile receiveBufferStart = receiveBuffer;
volatile uint8_t* volatile receiveBufferEnd = receiveBuffer;

typedef enum ServoCommand
{
   PING = 1,
   READ = 2,
   WRITE = 3
} ServoCommand;

void sendServoCommand(const uint8_t servoId, const ServoCommand commandByte, const uint8_t numParams, const uint8_t* params)
{
   sendServoByte(0xff);
   sendServoByte(0xff);  // command header

   sendServoByte(servoId);  // servo ID
   uint8_t checksum = servoId;

   sendServoByte(numParams + 2);  // number of following bytes
   sendServoByte((uint8_t)commandByte);  // command

   checksum += numParams + 2 + commandByte;

   for (uint8_t i = 0; i < numParams; i++)
   {
      sendServoByte(params[i]);  // parameters
      checksum += params[i];
   }

   sendServoByte(~checksum);  // checksum

   uint8_t tmp = (uint8_t)USART_ReceiveData(USART6); // grab the byte from the data register
}

bool getServoResponse(void)
{
   uint8_t retries = 0;

   clearServoReceiveBuffer();

   USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

   while (getServoBytesAvailable() < 4)
   {
      retries++;
      if (retries > REC_WAIT_MAX_RETRIES)
      {
#ifdef SERVO_DEBUG
         printf("Too many retries at start\n");
#endif
         return false;
      }

      delayus(REC_WAIT_START_US);
   }
   retries = 0;

   getServoByte();  // servo header (two 0xff bytes)
   getServoByte();

   response.id = getServoByte();
   response.length = getServoByte();

   if (response.length > SERVO_MAX_PARAMS)
   {
#ifdef SERVO_DEBUG
      printf("Response length too big: %d\n", (int)response.length);
#endif
      return false;
   }

   while (getServoBytesAvailable() < response.length)
   {
      retries++;
      if (retries > REC_WAIT_MAX_RETRIES)
      {
#ifdef SERVO_DEBUG
         printf("Too many retries waiting for params, got %d of %d params\n", getServoBytesAvailable(), response.length);
#endif
         return false;
      }

      delayus(REC_WAIT_START_US);
   }

   response.error = getServoByte();
   servoErrorCode = response.error;

   for (uint8_t i = 0; i < response.length - 2; i++)
   {
      response.params[i] = getServoByte();
   }

   uint8_t calcChecksum = response.id + response.length + response.error;

   for (uint8_t i = 0; i < response.length - 2; i++)
   {
      calcChecksum += response.params[i];
   }

   calcChecksum = ~calcChecksum;

   const uint8_t recChecksum = getServoByte();

   if (calcChecksum != recChecksum)
   {
#ifdef SERVO_DEBUG
      printf("Checksum mismatch: %x calculated, %x received\n", calcChecksum, recChecksum);
#endif
      return false;
   }

   return true;
}

bool getAndCheckResponse(const uint8_t servoId)
{
   if (!getServoResponse())
   {
#ifdef SERVO_DEBUG
      printf("Servo error: Servo %d did not respond correctly or at all\n", (int)servoId);
#endif
      USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);
      return false;
   }

   USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);

   if (response.id != servoId)
   {
#ifdef SERVO_DEBUG
      printf("Servo error: Response ID %d does not match command ID %d\n", (int)response.id);
#endif
      return false;
   }

   if (response.error != 0)
   {
#ifdef SERVO_DEBUG
      printf("Servo error: Response error code was nonzero (%d)\n", (int)response.error);
#endif
      return false;
   }

   return true;
}

// ping a servo, returns true if we get back the expected values
bool pingServo(const uint8_t servoId)
{
   sendServoCommand(servoId, PING, 0, 0);

   if (!getAndCheckResponse(servoId))
   {
      return false;
   }

   return true;
}

bool setServoReturnDelayMicros(const uint8_t servoId, const uint16_t micros)
{
   if (micros > 510)
      return false;

   const uint8_t params[2] = { RETURN_DELAY, (uint8_t)((micros / 2) & 0xff) };

   sendServoCommand(servoId, WRITE, 2, params);

   if (!getAndCheckResponse(servoId))
      return false;

   return true;
}

// set the events that will cause the servo to blink its LED
bool setServoBlinkConditions(const uint8_t servoId, const uint8_t flags)
{
   const uint8_t params[2] = { BLINK_CONDITIONS, flags };

   sendServoCommand(servoId, WRITE, 2, params);

   if (!getAndCheckResponse(servoId))
      return false;

   return true;
}

// set the events that will cause the servo to shut off torque
bool setServoShutdownConditions(const uint8_t servoId, const uint8_t flags)
{
   const uint8_t params[2] = { SHUTDOWN_CONDITIONS, flags };

   sendServoCommand(servoId, WRITE, 2, params);

   if (!getAndCheckResponse(servoId))
      return false;

   return true;
}

// valid torque values are from 0 (free running) to 1023 (max)
bool setServoTorque(const uint8_t servoId, const uint16_t torqueValue)
{
   const uint8_t highByte = (uint8_t)((torqueValue >> 8) & 0xff);
   const uint8_t lowByte = (uint8_t)(torqueValue & 0xff);

   if (torqueValue > 1023)
      return false;

   const uint8_t params[3] = { TORQUE, lowByte, highByte };

   sendServoCommand(servoId, WRITE, 3, params);

   if (!getAndCheckResponse(servoId))
      return false;

   return true;
}

bool getServoTorque(const uint8_t servoId, uint16_t* torqueValue)
{
   const uint8_t params[2] = { TORQUE, 2 };  // read two bytes, starting at address TORQUE

   sendServoCommand(servoId, READ, 2, params);

   if (!getAndCheckResponse(servoId))
      return false;

   *torqueValue = response.params[1];
   *torqueValue <<= 8;
   *torqueValue |= response.params[0];

   return true;
}

// speed values go from 1 (incredibly slow) to 1023 (114 RPM)
// a value of zero will disable velocity control
bool setServoMaxSpeed(const uint8_t servoId, const uint16_t speedValue)
{
   const uint8_t highByte = (uint8_t)((speedValue >> 8) & 0xff);
   const uint8_t lowByte = (uint8_t)(speedValue & 0xff);

   if (speedValue > 1023)
      return false;

   const uint8_t params[3] = { MAX_SPEED, lowByte, highByte };

   sendServoCommand(servoId, WRITE, 3, params);

   if (!getAndCheckResponse(servoId))
      return false;

   return true;
}

bool getServoMaxSpeed(const uint8_t servoId, uint16_t* speedValue)
{
   const uint8_t params[2] = { MAX_SPEED, 2 };  // read two bytes, starting at address MAX_SPEED

   sendServoCommand(servoId, READ, 2, params);

   if (!getAndCheckResponse(servoId))
   {
      return false;
   }

   *speedValue = response.params[1];
   *speedValue <<= 8;
   *speedValue |= response.params[0];

   return true;
}

bool getServoCurrentVelocity(const uint8_t servoId, int16_t* velocityValue)
{
   const uint8_t params[2] = { CURRENT_SPEED, 2 };  // read two bytes, starting at address CURRENT_SPEED

   sendServoCommand(servoId, READ, 2, params);

   if (!getAndCheckResponse(servoId))
   {
      return false;
   }

   *velocityValue = response.params[1];
   *velocityValue <<= 8;
   *velocityValue |= response.params[0];

   return true;
}

// make the servo move to an angle
// valid angles are between 0 and 300 degrees
bool setServoAngle(const uint8_t servoId, const float angle)
{
   if (angle < 0 || angle > 300)
   {
      return false;
   }

   // angle values go from 0 to 0x3ff (1023)
   const uint16_t angleValue = (uint16_t)(angle * (1023.0 / 300.0));

   const uint8_t highByte = (uint8_t)((angleValue >> 8) & 0xff);
   const uint8_t lowByte = (uint8_t)(angleValue & 0xff);

   const uint8_t params[3] = { GOAL_ANGLE, lowByte, highByte };

   sendServoCommand(servoId, WRITE, 3, params);

   if (!getAndCheckResponse(servoId))
   {
      return false;
   }

   return true;
}

bool getServoAngle(const uint8_t servoId, float* angle)
{
   const uint8_t params[2] = { CURRENT_ANGLE, 2 };  // read two bytes, starting at address CURRENT_ANGLE

   sendServoCommand(servoId, READ, 2, params);

   if (!getAndCheckResponse(servoId))
   {
      return false;
   }

   uint16_t angleValue = response.params[1];
   angleValue <<= 8;
   angleValue |= response.params[0];

   *angle = (float)angleValue * 300.0 / 1023.0;

   return true;
}

void sendServoByte(const uint8_t byte)
{
   USART_SendData(USART6, (uint16_t)byte);

   //Loop until the end of transmission
   while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
}

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

void USART6_IRQHandler(void)
{
   // check if the USART6 receive interrupt flag was set
   if (USART_GetITStatus(USART6, USART_IT_RXNE))
   {
      // GPIO_SetBits(GPIOA, GPIO_Pin_5); // Set C13 to High level ("1")
      // delayms(500);
      // GPIO_ResetBits(GPIOA, GPIO_Pin_5); // Set C13 to Low level ("0")

      static uint8_t count = 0;

      const uint8_t byte = (uint8_t)USART_ReceiveData(USART6); // grab the byte from the data register
      byte1 = byte;

      if (count < 20)
      {
         arr[count] = byte1;
         count++;
      }
      else
      {
         count = 0;
         arr[count] = byte1;
      }

      receiveBufferEnd++;
      if (receiveBufferEnd >= receiveBuffer + REC_BUFFER_LEN)
      {
         receiveBufferEnd = receiveBuffer;
      }

      *receiveBufferEnd = byte;
   }
}

void usartInit()
{
   GPIO_InitTypeDef GPIO_InitStructure;
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;

   /* Initialize USART6*/
   // enable the USART6 clock
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

   /* Configure USART6 Tx (PA.11) as alternate function push-pull */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   /* Configure USART6 Rx (PA.12) as input floating */ //or input with pull-up/-down
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
   //   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   //  GPIO_InitStructure.GPIO_OType = GPIO_OType;
   // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   // connect the output pin to the peripheral's alt function
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_USART6);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_USART6);

   USART_InitStructure.USART_BaudRate = 1000000;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

   USART_Init(USART6, &USART_InitStructure);

   // configure the USART6 interrupt
   NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   // NVIC_EnableIRQ(USART6_IRQn);

   // enable the USART6 receive interrupt
   USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

   /* Enable USART6 */
   USART_Cmd(USART6, ENABLE);
}

void usartHalfDuplexInit()
{
   GPIO_InitTypeDef GPIO_InitStructure;
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;

   clearServoReceiveBuffer();

   /* Initialize USART6*/
   // enable the USART6 clock
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

   // Configure USART6 Tx (PA.11)
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   // GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   // connect the output pin to the peripheral's alt function
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_USART6);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_USART6);

   USART_InitStructure.USART_BaudRate = 1000000;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

   USART_Init(USART6, &USART_InitStructure);

   // configure the USART6 interrupt
   NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   // enable the USART6 receive interrupt
   USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

   // set USART6 to half-duplex
   USART_HalfDuplexCmd(USART6, ENABLE);

   /* Enable USART6 */
   USART_Cmd(USART6, ENABLE);
}

void init()
{
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

void sendByteln(char c)
{
   USART_SendData(USART6, c);

   while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
   {

   }

   USART_SendData(USART6, '\n');

   while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
   {

   }
}

void sendByte(uint8_t byte)
{
   USART_SendData(USART6, byte);

   while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
   {

   }
}

void sendByteArray(uint8_t* p, uint8_t length)
{
   for (uint8_t i = 0; i < length; i++)
   {
      sendByte(p[i]);
   }
}

void turn(unsigned char ID, int Speed)
{
   char Speed_H, Speed_L;
   Speed_H = Speed >> 8;
   Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables

   const unsigned int length = 9;
   unsigned char packet[length];

   Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H)) & 0xFF;

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = ID;
   packet[3] = AX_SPEED_LENGTH;
   packet[4] = AX_WRITE_DATA;
   packet[5] = AX_GOAL_SPEED_L;
   packet[6] = Speed_L;
   packet[7] = Speed_H;
   packet[8] = Checksum;

   // packet[0] = 0xFF;
   // packet[1] = 0xFF;
   // packet[2] = 0x01;
   // packet[3] = 0x05;
   // packet[4] = 0x03;
   // packet[5] = 0x20;
   // packet[6] = 0x64;
   // packet[7] = 0x00;
   // packet[8] = 0x72;

   sendByteArray(packet, 9);
}

void setEndless(unsigned char ID, bool Status)
{
   const unsigned int length = 9;
   unsigned char packet[length];

   Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L)) & 0xFF;

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = ID;
   packet[3] = AX_GOAL_LENGTH;
   packet[4] = AX_WRITE_DATA;
   packet[5] = AX_CCW_ANGLE_LIMIT_L;
   packet[6] = 0; 						// full rotation
   packet[7] = 0;						// full rotation
   packet[8] = Checksum;

   // packet[0] = 0xFF;
   // packet[1] = 0xFF;
   // packet[2] = 0x01;
   // packet[3] = 0x05;
   // packet[4] = 0x03;
   // packet[5] = 0x08;
   // packet[6] = 0x00;
   // packet[7] = 0x00;
   // packet[8] = 0xEE;

   sendByteArray(packet, 9);
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
   changeCoreFrequency();

   __disable_irq();

   systemTimeInit();

   init();

   // usartInit();
   usartHalfDuplexInit();

   __enable_irq();

   //   GPIO_ResetBits(GPIOA, GPIO_Pin_5); // Set C13 to Low level ("0")

   USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);

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


