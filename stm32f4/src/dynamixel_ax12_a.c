/***********************************************
*	Dynamixel AX12-A
*	Version 0.1
************************************************/

#include "dynamixel_ax12_a.h"

unsigned char Checksum = 0;
unsigned long delta = 0;
uint8_t servoErrorCode = 0;
uint8_t id = 0;
uint8_t byte1 = 0;
bool flag = 0;
uint8_t arr[20];
volatile uint8_t receiveBuffer[REC_BUFFER_LEN];
volatile uint8_t* volatile receiveBufferStart = receiveBuffer;
volatile uint8_t* volatile receiveBufferEnd = receiveBuffer;

ServoResponse response;

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

bool pingServo(const uint8_t servoId)
{
	sendServoCommand(servoId, PING, 0, 0);

	if (!getAndCheckResponse(servoId))
	{
		return false;
	}

	return true;
}

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

void sendServoByte(const uint8_t byte)
{
	USART_SendData(USART6, (uint16_t)byte);

	//Loop until the end of transmission
	while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);

	int a = 0;
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

void turn(unsigned char ID, int16_t speed)
{
	uint8_t speed_H, speed_L;
	speed_H = speed >> 8;
	speed_L = speed;                     // 16 bits - 2 x 8 bits variables

	const unsigned int length = 9;
	unsigned char packet[length];

	Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + speed_L + speed_H)) & 0xFF;

	packet[0] = AX_START;
	packet[1] = AX_START;
	packet[2] = ID;
	packet[3] = AX_SPEED_LENGTH;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_GOAL_SPEED_L;
	packet[6] = speed_L;
	packet[7] = speed_H;
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

	sendByteArray(packet, length);
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

	sendByteArray(packet, length);
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



