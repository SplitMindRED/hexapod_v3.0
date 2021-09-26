/***********************************************
*	Dynamixel AX12-A
*	Version 0.1
************************************************/

#include "hal_dynamixel_ax12_a.h"

unsigned long delta = 0;
uint8_t servoErrorCode = 0;
bool flag = 0;
volatile uint8_t receiveBuffer[REC_BUFFER_LEN];
volatile uint8_t *volatile receiveBufferStart = receiveBuffer;
volatile uint8_t *volatile receiveBufferEnd = receiveBuffer;

bool pingServo(uint8_t servo_id)
{
   unsigned char packet[6];
   unsigned char checksum;

   ServoResponse response;

   uint8_t answer[20];

   for (uint8_t i = 0; i < 20; i++)
   {
      answer[i] = 0;
   }

   checksum = ~(servo_id + AX_PING_LENGTH + AX_PING);

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = servo_id;
   packet[3] = AX_PING_LENGTH;
   packet[4] = AX_PING;
   packet[5] = checksum;

   HAL_UART_Transmit(UART1, packet, sizeof(packet), MAX_DELAY);

   if (HAL_UART_Receive(UART1, answer, 7, MAX_DELAY) != HAL_OK)
   {
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" recieve ping FAIL!");
      turnLed(1);
      return false;
   }

   response.id = answer[3];
   response.length = answer[4];
   response.error = answer[5];

   for (uint8_t i = 0; i < (response.length - 2); i++)
   {
      response.params[0] = answer[6 + i];
   }

   response.checksum = answer[6 + (response.length - 2)];

   uint16_t param_sum = 0;

   for (uint8_t i = 0; i < (response.length - 2); i++)
   {
      param_sum += response.params[i];
   }

   checksum = ~(response.id + response.length + response.error + param_sum);

   if ((checksum == response.checksum) && (response.id == servo_id))
   {
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" ping SUCCSESS!");
      turnLed(0);
      return true;
   }
   else
   {
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" ping FAIL!");
      turnLed(1);
      return false;
   }
}

void setVelocity(uint8_t servo_id, int16_t velocity)
{
   uint8_t velocity_L, velocity_H;
   velocity_H = velocity >> 8;
   velocity_L = velocity;                     // 16 bits - 2 x 8 bits variables
   unsigned char checksum;

   unsigned char packet[9];

   checksum = (~(servo_id + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + velocity_L + velocity_H));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = servo_id;
   packet[3] = AX_SPEED_LENGTH;
   packet[4] = AX_WRITE_DATA;
   packet[5] = AX_GOAL_SPEED_L;
   packet[6] = velocity_L;
   packet[7] = velocity_H;
   packet[8] = checksum;

   // packet[0] = 0xFF;
   // packet[1] = 0xFF;
   // packet[2] = 0x01;
   // packet[3] = 0x05;
   // packet[4] = 0x03;
   // packet[5] = 0x20;
   // packet[6] = 0x64;
   // packet[7] = 0x00;
   // packet[8] = 0x72;

   HAL_UART_Transmit(UART1, packet, sizeof(packet), MAX_DELAY);
}

void wheelMode(uint8_t servo_id, bool status)
{
   unsigned char packet[9];
   unsigned char checksum;

   checksum = (~(servo_id + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = servo_id;
   packet[3] = AX_GOAL_LENGTH;
   packet[4] = AX_WRITE_DATA;
   packet[5] = AX_CCW_ANGLE_LIMIT_L;
   packet[6] = 0; 						// full rotation
   packet[7] = 0;						   // full rotation
   packet[8] = checksum;

   // packet[0] = 0xFF;
   // packet[1] = 0xFF;
   // packet[2] = 0x01;
   // packet[3] = 0x05;
   // packet[4] = 0x03;
   // packet[5] = 0x08;
   // packet[6] = 0x00;
   // packet[7] = 0x00;
   // packet[8] = 0xEE;

   HAL_UART_Transmit(UART1, packet, sizeof(packet), MAX_DELAY);
}

// bool changeID(uint8_t new_id)
// {
// 	const unsigned int length = 8;
// 	unsigned char packet[length];

// 	checksum = (~(BROADCAST_ID + AX_ID_LENGTH + AX_WRITE_DATA + AX_ID + new_id));

// 	packet[0] = AX_START;
// 	packet[1] = AX_START;
// 	packet[2] = BROADCAST_ID;
// 	packet[3] = AX_ID_LENGTH;
// 	packet[4] = AX_WRITE_DATA;
// 	packet[5] = AX_ID;
// 	packet[6] = new_id;
// 	packet[7] = checksum;

// 	sendByteArray(packet, length);

// 	delayms(100);

// 	if (pingServo(new_id))
// 	{
// 		return true;
// 	}
// 	else
// 	{
// 		return false;
// 	}
// }

int16_t getAngle(uint8_t servo_id)
{
   unsigned char packet[8];
   unsigned char checksum = 0;

   ServoResponse response;

   uint8_t answer[20];

   for (uint8_t i = 0; i < 20; i++)
   {
      answer[i] = 0;
   }

   checksum = (~(servo_id + AX_POS_LENGTH + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_BYTE_READ_POS));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = servo_id;
   packet[3] = AX_POS_LENGTH;
   packet[4] = AX_READ_DATA;
   packet[5] = AX_PRESENT_POSITION_L;
   packet[6] = AX_BYTE_READ_POS;
   packet[7] = checksum;

   HAL_UART_Transmit(UART1, packet, sizeof(packet), MAX_DELAY);

   if (HAL_UART_Receive(UART1, answer, 9, MAX_DELAY) != HAL_OK)
   {
      return -1;
   }

   response.id = answer[3];
   response.length = answer[4];
   response.error = answer[5];

   for (uint8_t i = 0; i < (response.length - 2); i++)
   {
      response.params[0 + i] = answer[6 + i];
   }

   response.checksum = answer[6 + (response.length - 2)];

   uint16_t param_sum = 0;

   for (uint8_t i = 0; i < (response.length - 2); i++)
   {
      param_sum += response.params[i];
   }

   checksum = ~(response.id + response.length + response.error + param_sum);

   if ((checksum == response.checksum) && (response.id == servo_id))
   {
      uint16_t pos = response.params[0] | (response.params[1] << 8);
      return pos;
   }
   else
   {
      UART_printStrLn("Read pos FAIL!");
      return -1;
   }
}

void jointMode(uint8_t id)
{
   unsigned char packet[9];
   uint8_t limit_L = 1023;
   uint8_t limit_H = 1023 >> 8;
   unsigned char checksum = 0;

   checksum = (~(id + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L + limit_L + limit_H));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = id;
   packet[3] = AX_GOAL_LENGTH;
   packet[4] = AX_WRITE_DATA;
   packet[5] = AX_CCW_ANGLE_LIMIT_L;
   packet[6] = limit_L;
   packet[7] = limit_H;
   packet[8] = checksum;

   HAL_UART_Transmit(UART1, packet, sizeof(packet), MAX_DELAY);
}

void setAngle(uint8_t servo_id, uint16_t angle)
{
   unsigned char packet[9];
   uint8_t angle_L = angle;
   uint8_t angle_H = angle >> 8;
   unsigned char checksum = 0;

   checksum = (~(servo_id + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + angle_L + angle_H));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = servo_id;
   packet[3] = AX_GOAL_LENGTH;
   packet[4] = AX_WRITE_DATA;
   packet[5] = AX_GOAL_POSITION_L;
   packet[6] = angle_L;
   packet[7] = angle_H;
   packet[8] = checksum;

   HAL_UART_Transmit(UART1, packet, sizeof(packet), MAX_DELAY);
}

