/***********************************************
*	Dynamixel AX12-A
*	Version 0.1
************************************************/

#include "hal_dynamixel_ax12_a.h"

Servo servo[2];

int8_t initAllDynamixel(void)
{
   servo[0].id = 1;
   servo[1].id = 2;
}

int8_t pingServo(uint8_t servo_id)
{
   unsigned char packet[6];
   unsigned char checksum;

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
#ifdef U1_DEBUG
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" recieve ping FAIL!");
#endif
      turnLed(1);
      return ERROR;
   }

   ServoResponse response = checkResponse(servo_id, answer);

   if (response.result == OK)
   {
#ifdef U1_DEBUG
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" ping SUCCSESS!");
#endif
      turnLed(0);
      return OK;
   }
   else
   {
#ifdef U1_DEBUG
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" ping FAIL!");
#endif      
      turnLed(1);
      return ERROR;
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

int8_t wheelMode(uint8_t servo_id, bool status)
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

   // HAL_UART_Transmit(UART1, packet, sizeof(packet), MAX_DELAY);
   if (HAL_UART_Transmit(UART1, packet, sizeof(packet), MAX_DELAY) == HAL_OK)
   {
      servo[servo_id].mode = WHEEL_MODE;
      return OK;
   }
   else
   {
      return ERROR;
   }
}

int16_t getAngle(uint8_t servo_id)
{
   unsigned char packet[8];
   unsigned char checksum = 0;

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
      return ERROR;
   }

   ServoResponse response = checkResponse(servo_id, answer);

   if (response.result == OK)
   {
      uint16_t pos = response.params[0] | (response.params[1] << 8);
      servo[servo_id].angle = pos;
      return pos;
   }
   else
   {
#ifdef U1_DEBUG
      UART_printStrLn("Read pos FAIL!");
#endif
      return ERROR;
   }
}

int8_t jointMode(uint8_t servo_id)
{
   unsigned char packet[9];
   uint8_t limit_L = 1023;
   uint8_t limit_H = 1023 >> 8;
   unsigned char checksum = 0;

   checksum = (~(servo_id + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L + limit_L + limit_H));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = servo_id;
   packet[3] = AX_GOAL_LENGTH;
   packet[4] = AX_WRITE_DATA;
   packet[5] = AX_CCW_ANGLE_LIMIT_L;
   packet[6] = limit_L;
   packet[7] = limit_H;
   packet[8] = checksum;

   if (HAL_UART_Transmit(UART1, packet, sizeof(packet), MAX_DELAY) == HAL_OK)
   {
      servo[servo_id].mode = JOINT_MODE;
      return OK;
   }
   else
   {
      return ERROR;
   }
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

ServoResponse checkResponse(uint8_t servo_id, uint8_t *p_answer)
{
   ServoResponse response;
   unsigned char checksum = 0;

   response.id = p_answer[3];
   response.length = p_answer[4];
   response.error = p_answer[5];

   for (uint8_t i = 0; i < (response.length - 2); i++)
   {
      response.params[0 + i] = p_answer[6 + i];
   }

   response.checksum = p_answer[6 + (response.length - 2)];

   uint16_t param_sum = 0;

   for (uint8_t i = 0; i < (response.length - 2); i++)
   {
      param_sum += response.params[i];
   }

   checksum = ~(response.id + response.length + response.error + param_sum);

   if ((checksum == response.checksum) && (response.id == servo_id))
   {
      response.result = OK;
   }
   else
   {
      response.result = ERROR;
   }

   return response;
}

int16_t getVelocity(uint8_t servo_id)
{
   unsigned char packet[8];
   unsigned char checksum = 0;

   uint8_t answer[20];

   for (uint8_t i = 0; i < 20; i++)
   {
      answer[i] = 0;
   }

   checksum = (~(servo_id + AX_POS_LENGTH + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_BYTE_READ_POS));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = servo_id;
   packet[3] = AX_SPEED_LENGTH;
   packet[4] = AX_READ_DATA;
   packet[5] = AX_PRESENT_SPEED_L;
   packet[6] = 2;
   packet[7] = checksum;

   if (HAL_UART_Transmit(UART1, packet, sizeof(packet), MAX_DELAY) != OK)
   {
      return ERROR;
   }

   if (HAL_UART_Receive(UART1, answer, 9, MAX_DELAY) != HAL_OK)
   {
      return ERROR;
   }

   ServoResponse response = checkResponse(servo_id, answer);

   if (response.result == OK)
   {
      uint16_t vel = response.params[0] | (response.params[1] << 8);
      servo[servo_id].velocity = vel;

      if ((vel & 1 << 10))
      {
         vel &= ~(1 << 10);
         vel = -vel;
      }

      return vel;
   }
   else
   {
#ifdef U1_DEBUG
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" read velocity FAIL!");
#endif
      return ERROR;
   }
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
