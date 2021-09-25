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

uint8_t arr[20];

// void USART6_IRQHandler(void)
// {
// 	// check if the USART6 receive interrupt flag was set
// 	if (USART_GetITStatus(USART6, USART_IT_RXNE))
// 	{
// 		static uint8_t count = 0;

// 		const uint8_t byte = (uint8_t)USART_ReceiveData(USART6); // grab the byte from the data register
// 		byte1 = byte;

// 		if (count < 20)
// 		{
// 			arr[count] = byte1;
// 			count++;
// 		}
// 		else
// 		{
// 			count = 0;
// 			arr[count] = byte1;
// 		}

// 		receiveBufferEnd++;
// 		if (receiveBufferEnd >= receiveBuffer + REC_BUFFER_LEN)
// 		{
// 			receiveBufferEnd = receiveBuffer;
// 		}

// 		*receiveBufferEnd = byte;
// 	}
// }

// void USART1_IRQHandler(void)
// {
// 	// check if the USART1 receive interrupt flag was set
// 	if (USART_GetITStatus(USART1, USART_IT_RXNE))
// 	{
// 		static uint8_t count = 0;

// 		const uint8_t byte = (uint8_t)USART_ReceiveData(USART1); // grab the byte from the data register
// 		byte1 = byte;

// 		if (count < 20)
// 		{
// 			arr[count] = byte1;
// 			count++;
// 		}
// 		else
// 		{
// 			count = 0;
// 			arr[count] = byte1;
// 		}

// 		receiveBufferEnd++;
// 		if (receiveBufferEnd >= receiveBuffer + REC_BUFFER_LEN)
// 		{
// 			receiveBufferEnd = receiveBuffer;
// 		}

// 		*receiveBufferEnd = byte;
// 	}
// }

bool pingServo(UART_HandleTypeDef *huart, const uint8_t servo_id)
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

   HAL_UART_Transmit(huart, packet, sizeof(packet), HAL_MAX_DELAY);

   if (HAL_UART_Receive(huart, answer, 7, HAL_MAX_DELAY) != HAL_OK)
   {
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
      UART_printStrLn("Ping SUCCSESS!");
      return true;
   }
   else
   {
      UART_printStrLn("Ping FAIL!");
      return false;
   }
}

// void sendServoCommand(const uint8_t servo_id, const ServoCommand commandByte, const uint8_t numParams, const uint8_t* params)
// {
// 	sendServoByte(0xff);
// 	sendServoByte(0xff);  // command header

// 	sendServoByte(servo_id);  // servo ID
// 	uint8_t checksum = servo_id;

// 	sendServoByte(numParams + 2);  // number of following bytes
// 	sendServoByte((uint8_t)commandByte);  // command

// 	checksum += numParams + 2 + commandByte;

// 	for (uint8_t i = 0; i < numParams; i++)
// 	{
// 		sendServoByte(params[i]);  // parameters
// 		checksum += params[i];
// 	}

// 	sendServoByte(~checksum);  // checksum

// 	uint8_t tmp = (uint8_t)USART_ReceiveData(USART6); // grab the byte from the data register
// }

// void sendServoByte(const uint8_t byte)
// {
// 	USART_SendData(USART6, (uint16_t)byte);

// 	//Loop until the end of transmission
// 	while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
// }

// bool getServoResponse(void)
// {
// 	uint8_t retries = 0;

// 	clearServoReceiveBuffer();

// 	// USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
// 	// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

// 	while (getServoBytesAvailable() < 4)
// 	{
// 		retries++;
// 		if (retries > REC_WAIT_MAX_RETRIES)
// 		{
// #ifdef SERVO_DEBUG
// 			printf("Too many retries at start\n");
// #endif
// 			return false;
// 		}

// 		delayus(REC_WAIT_START_US);
// 	}
// 	retries = 0;

// 	getServoByte();  // servo header (two 0xff bytes)
// 	getServoByte();

// 	response.id = getServoByte();
// 	response.length = getServoByte();

// 	if (response.length > SERVO_MAX_PARAMS)
// 	{
// #ifdef SERVO_DEBUG
// 		printf("Response length too big: %d\n", (int)response.length);
// #endif
// 		return false;
// 	}

// 	while (getServoBytesAvailable() < response.length)
// 	{
// 		retries++;
// 		if (retries > REC_WAIT_MAX_RETRIES)
// 		{
// #ifdef SERVO_DEBUG
// 			printf("Too many retries waiting for params, got %d of %d params\n", getServoBytesAvailable(), response.length);
// #endif
// 			return false;
// 		}

// 		delayus(REC_WAIT_START_US);
// 	}

// 	response.error = getServoByte();
// 	servoErrorCode = response.error;

// 	for (uint8_t i = 0; i < response.length - 2; i++)
// 	{
// 		response.params[i] = getServoByte();
// 	}

// 	uint8_t calcchecksum = response.id + response.length + response.error;

// 	for (uint8_t i = 0; i < response.length - 2; i++)
// 	{
// 		calcchecksum += response.params[i];
// 	}

// 	calcchecksum = ~calcchecksum;

// 	const uint8_t recchecksum = getServoByte();

// 	if (calcchecksum != recchecksum)
// 	{
// #ifdef SERVO_DEBUG
// 		printf("checksum mismatch: %x calculated, %x received\n", calcchecksum, recchecksum);
// #endif
// 		return false;
// 	}

// 	return true;
// }

// bool getAndCheckResponse(const uint8_t servo_id)
// {
// 	if (!getServoResponse())
// 	{
// #ifdef SERVO_DEBUG
// 		printf("Servo error: Servo %d did not respond correctly or at all\n", (int)servo_id);
// #endif
// 		// USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);
// 		// USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

// 		return false;
// 	}

// 	// USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);
// 	// USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

// 	if (response.id != servo_id)
// 	{
// #ifdef SERVO_DEBUG
// 		printf("Servo error: Response ID %d does not match command ID %d\n", (int)response.id);
// #endif
// 		return false;
// 	}

// 	if (response.error != 0)
// 	{
// #ifdef SERVO_DEBUG
// 		printf("Servo error: Response error code was nonzero (%d)\n", (int)response.error);
// #endif
// 		return false;
// 	}

// 	return true;
// }

void turn(UART_HandleTypeDef *huart, unsigned char ID, int16_t speed)
{
   uint8_t speed_H, speed_L;
   speed_H = speed >> 8;
   speed_L = speed;                     // 16 bits - 2 x 8 bits variables
   unsigned char checksum;

   unsigned char packet[9];

   checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + speed_L + speed_H));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = ID;
   packet[3] = AX_SPEED_LENGTH;
   packet[4] = AX_WRITE_DATA;
   packet[5] = AX_GOAL_SPEED_L;
   packet[6] = speed_L;
   packet[7] = speed_H;
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

   HAL_UART_Transmit(huart, packet, sizeof(packet), HAL_MAX_DELAY);
}

void setEndless(UART_HandleTypeDef *huart, unsigned char ID, bool status)
{
   unsigned char packet[9];
   unsigned char checksum;

   checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = ID;
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

   HAL_UART_Transmit(huart, packet, sizeof(packet), HAL_MAX_DELAY);
}

// void clearServoReceiveBuffer(void)
// {
// 	receiveBufferStart = receiveBufferEnd;
// }

// uint8_t getServoBytesAvailable(void)
// {
// 	volatile uint8_t* start = receiveBufferStart;
// 	volatile uint8_t* end = receiveBufferEnd;

// 	if (end >= start)
// 	{
// 		return (uint8_t)(end - start);
// 	}
// 	else
// 	{
// 		return (uint8_t)(REC_BUFFER_LEN - (start - end));
// 	}
// }

// uint8_t getServoByte(void)
// {
// 	receiveBufferStart++;
// 	if (receiveBufferStart >= receiveBuffer + REC_BUFFER_LEN)
// 	{
// 		receiveBufferStart = receiveBuffer;
// 	}

// 	return *receiveBufferStart;
// }

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

void getActualPosition(UART_HandleTypeDef *huart, uint8_t servo_id)
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

   HAL_UART_Transmit(huart, packet, sizeof(packet), HAL_MAX_DELAY);

   if (HAL_UART_Receive(huart, answer, 9, HAL_MAX_DELAY) != HAL_OK)
   {
      return;
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
      // UART_printStrLn("Ping SUCCSESS!");
      uint16_t pos = response.params[0] | (response.params[1] << 8);
      UART_printStr("Pos: ");
      UART_printLn(pos);
      return;
   }
   else
   {
      int a = 0;
      // UART_printStrLn("Ping FAIL!");
      return;
   }
}

// void jointMode(uint8_t id)
// {
// 	const unsigned int length = 9;
// 	unsigned char packet[length];
// 	uint8_t limit_l = 1023;
// 	uint8_t limit_h = 1023 >> 8;

// 	checksum = (~(id + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L + limit_l + limit_h));

// 	packet[0] = AX_START;
// 	packet[1] = AX_START;
// 	packet[2] = id;
// 	packet[3] = AX_GOAL_LENGTH;
// 	packet[4] = AX_WRITE_DATA;
// 	packet[5] = AX_CCW_ANGLE_LIMIT_L;
// 	packet[6] = limit_l;
// 	packet[7] = limit_h;
// 	packet[8] = checksum;

// 	sendByteArray1(packet, length);
// }

// void setAngle(uint8_t id, uint16_t angle)
// {
// 	const unsigned int length = 9;
// 	unsigned char packet[length];
// 	uint8_t angle_l = angle;
// 	uint8_t angle_h = angle >> 8;

// 	checksum = (~(id + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + angle_l + angle_h));

// 	packet[0] = AX_START;
// 	packet[1] = AX_START;
// 	packet[2] = id;
// 	packet[3] = AX_GOAL_LENGTH;
// 	packet[4] = AX_WRITE_DATA;
// 	packet[5] = AX_GOAL_POSITION_L;
// 	packet[6] = angle_l;
// 	packet[7] = angle_h;
// 	packet[8] = checksum;

// 	sendByteArray1(packet, length);
// }

