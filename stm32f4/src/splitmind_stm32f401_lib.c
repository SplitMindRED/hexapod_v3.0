#include "splitmind_stm32f401_lib.h"

volatile unsigned long system_time = 0;

// //set mode of pin of port. look to define for parameters
// void pinMode(uint8_t port, uint8_t pin, uint8_t mode, uint8_t config)
// {
// 	//enable clock on port
// 	RCC->APB2ENR |= 1 << port;
// 	//CRL
// 	if (pin <= 7)
// 	{
// 		//reset config
// 		((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->CRL &= ~(1 << (pin * 4 + 2));
// 		//set mode and config
// 		((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->CRL |= mode << (pin * 4);
// 		((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->CRL |= config << (pin * 4 + 2);
// 	}
// 	//CRH
// 	else
// 	{
// 		//reset config
// 		((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->CRH &= ~(1 << (((pin - 8) * 4) + 2));
// 		//set mode and config
// 		((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->CRH |= mode << ((pin - 8) * 4);
// 		((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->CRH |= config << (((pin - 8) * 4) + 2);
// 	}
// }

//Send 0 or 1 to pin
void digitalWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool value)
{
	if (value == 1)
	{
		GPIO_SetBits(GPIOx, GPIO_Pin); // Set pin to Low level ("0")
	}
	else if (value == 0)
	{
		GPIO_ResetBits(GPIOx, GPIO_Pin); // Set pin to Low level ("0")
	}
}

void changeCoreFrequency()
{
	FLASH->ACR |= FLASH_ACR_PRFTEN;
	FLASH->ACR &= ~(FLASH_ACR_LATENCY);
	FLASH->ACR |= FLASH_ACR_LATENCY_2WS;

	//clear register
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN);

	//PLLP = 8 (11)
	RCC->PLLCFGR |= 1 << 16 | 1 << 17;

	//PLLN = 210 (1101 0010) from 14 to 6 bit
	RCC->PLLCFGR |= 1 << 7 | 1 << 10 | 1 << 12 | 1 << 13;

	//PLLM = 5 (101)
	RCC->PLLCFGR |= 1 << 0 | 1 << 2;

	RCC->CFGR &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_HPRE);
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_HPRE_DIV1;

	//PLL on
	RCC->CR |= 1 << 24;
	while (!(RCC->CR & RCC_CR_PLLRDY));

	//system core switch to PLL
	RCC->CFGR |= 1 << 1;
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));

	SystemCoreClockUpdate();
}

bool delayms(int millisec)
{
	unsigned long next_time = system_time + (millisec * SYSTICKFREQ / 1000);

	while (system_time <= next_time)
	{
		//waiting
	}

	return 1;
}

bool delayus(int microsec)
{
	unsigned long next_time = system_time + microsec;

	while (system_time <= next_time)
	{
		//waiting
	}

	return 1;
}

// bool digitalRead(uint8_t port, uint8_t pin)
// {
// 	bool value;

// 	value = ((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->IDR & (1 << pin);

// 	return value;
// }

// //hard delay, empty cycle
// void delay(int millisec)
// {
// 	unsigned long start_time = system_time;
// 	while (system_time <= (start_time + (millisec * 1000)))
// 	{
// 		//waiting
// 	}
// }

// //count pulse length and return
// uint64_t pulseIN(uint8_t PIN)
// {
// 	uint16_t pulse_length = 0;
// 	unsigned long start_count = 0;
// 	bool high_signal = false;

// 	while (1)
// 	{
// 		//when comes 1 -> start count
// 		if (GPIOA->IDR & (1 << (PIN)) && high_signal == false)
// 		{
// 			start_count = system_time;
// 			high_signal = true;
// 		}

// 		//when comes 0 -> stop count and exit from cycle
// 		if ((high_signal == true) && !(GPIOA->IDR & (1 << (PIN))))
// 		{
// 			high_signal = false;
// 			pulse_length = system_time - start_count;
// 			break;
// 		}
// 	}

// 	delay(1);

// 	return pulse_length;
// }

// float map(float have, float have_min, float have_max, float need_min, float need_max)
// {
// 	float ratio = 0;
// 	float add = 0;

// 	ratio = (need_max - need_min) / (have_max - have_min);
// 	add = need_max - (have_max * ratio);


// 	return (have * ratio + add);
// }

// bool checkTimer(SoftTimer_ms* timer)
// {
// 	if (system_time >= (timer->start_time + (timer->delay * 1000)))
// 	{
// 		timer->start_time = system_time;
// 		return true;
// 	}
// 	else
// 	{
// 		return false;
// 	}
// }

// //I2C----------------------------------------------------------------------------------------------------
// GPIO_InitTypeDef GPIO_InitStructure;

// void I2C1_init(void)
// {
// 	I2C_InitTypeDef  I2C_InitStructure;
// 	GPIO_InitTypeDef  GPIO_InitStructure;

// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

// 	/* Configure I2C_EE pins: SCL and SDA */
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);

// 	/* I2C configuration */
// 	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
// 	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
// 	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
// 	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
// 	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
// 	I2C_InitStructure.I2C_ClockSpeed = 400000;

// 	/* I2C Peripheral Enable */
// 	I2C_Cmd(I2C1, ENABLE);
// 	/* Apply I2C configuration after enabling it */
// 	I2C_Init(I2C1, &I2C_InitStructure);
// }

// void I2C_writeByte(uint8_t device_address, uint8_t address, uint8_t data)
// {
// 	uint32_t stop = 0;

// 	//send START
// 	I2C_GenerateSTART(I2C1, ENABLE);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	//send slave address
// 	I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Transmitter);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	//send register address
// 	I2C_SendData(I2C1, address);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	//send data
// 	I2C_SendData(I2C1, data);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	//send STOP
// 	I2C_GenerateSTOP(I2C1, ENABLE);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			break;
// 		}
// 	}
// }

// void I2C_burstWrite(uint8_t device_address, uint8_t address, uint8_t n_data, uint8_t* data)
// {
// 	I2C_GenerateSTART(I2C1, ENABLE);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

// 	I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Transmitter);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

// 	I2C_SendData(I2C1, address);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

// 	while (n_data--)
// 	{
// 		I2C_SendData(I2C1, *data++);
// 		n_data--;
// 		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
// 	}

// 	I2C_GenerateSTOP(I2C1, ENABLE);
// 	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
// }

// uint8_t I2C_readByte(uint8_t device_address, uint8_t address)
// {
// 	uint8_t data;
// 	uint32_t stop = 0;

// 	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	I2C_GenerateSTART(I2C1, ENABLE);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Transmitter);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	I2C_SendData(I2C1, address);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	I2C_GenerateSTART(I2C1, ENABLE);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Receiver);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}
// 	data = I2C_ReceiveData(I2C1);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	I2C_AcknowledgeConfig(I2C1, DISABLE);
// 	I2C_GenerateSTOP(I2C1, ENABLE);
// 	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	return data;
// }

// void I2C_burstRead(uint8_t device_address, uint8_t address, uint8_t n_data, uint8_t* data)
// {
// 	uint32_t stop = 0;

// 	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	I2C_GenerateSTART(I2C1, ENABLE);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Transmitter);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	I2C_SendData(I2C1, address);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	I2C_GenerateSTOP(I2C1, ENABLE);

// 	I2C_GenerateSTART(I2C1, ENABLE);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Receiver);
// 	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}

// 	I2C_AcknowledgeConfig(I2C1, ENABLE);
// 	while (n_data--)
// 	{
// 		if (!n_data)
// 		{
// 			I2C_AcknowledgeConfig(I2C1, DISABLE);
// 		}
// 		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
// 		{
// 			stop++;
// 			if (stop >= 1000)
// 			{
// 				stop = 0;
// 				break;
// 			}
// 		}

// 		*data++ = I2C_ReceiveData(I2C1);
// 	}

// 	I2C_AcknowledgeConfig(I2C1, DISABLE);
// 	I2C_GenerateSTOP(I2C1, ENABLE);
// 	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
// 	{
// 		stop++;
// 		if (stop >= 1000)
// 		{
// 			stop = 0;
// 			break;
// 		}
// 	}
// }
// //END OF I2C---------------------------------------------------------------------------------------------

// //SPI----------------------------------------------------------------------------------------------------
// void SPI_init(void)
// {
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

// 	//Fclk = 72 MHz
// 	//BR[2]
// 	//000 - Fclk/2		36 MHz
// 	//001 - Fclk/4		18 MHz
// 	//010 - Fclk/8		9 MHz
// 	//011 - Fclk/16	4.5 MHz
// 	//100 - Fclk/32	2.25 MHz
// 	//101 - Fclk/64	1.125 MHz
// 	//110 - Fclk/128	562.5 kHz
// 	//111 - Fclk/256	281.25 kHz

// 	//SPI1->CR1 |= SPI_CR1_BR_0;	
// 	//SPI1->CR1 |= SPI_CR1_BR_1;	
// 	SPI1->CR1 |= SPI_CR1_BR_2;

// 	SPI1->CR1 |= SPI_CR1_MSTR;		//master mode	
// 	SPI1->CR2 |= SPI_CR2_SSOE;		//SS output enable
// 	SPI1->CR1 |= SPI_CR1_SPE;		//SPI enable

// 	//slave acknowledge pin
// //	pinMode(PORT_A, 3, INPUT, INPUT_PULL_UP_DOWN);
// 	pinMode(PORT_A, 8, INPUT, INPUT_PULL_UP_DOWN);

// 	//NSS
// 	pinMode(PORT_A, 4, OUTPUT_50, OUTPUT_GPO_PUSH_PULL);
// 	//SCK
// 	pinMode(PORT_A, 5, OUTPUT_50, OUTPUT_AF_PUSH_PULL);
// 	//MISO
// 	pinMode(PORT_A, 6, INPUT, INPUT_PULL_UP_DOWN);
// 	//MOSI
// 	pinMode(PORT_A, 7, OUTPUT_50, OUTPUT_AF_PUSH_PULL);
// }

// uint8_t SPI_Transfer(uint8_t data)
// {
// 	uint8_t recieved_value = 0;

// 	// Write data to be transmitted to the SPI data register
// 	SPI1->DR = data;

// 	// Wait until data gets form TX buffer to shift register
// 	while (!(SPI1->SR & (SPI_I2S_FLAG_TXE)));

// 	// Wait until transfer finishes
// 	while (!(SPI1->SR & (SPI_I2S_FLAG_RXNE)));

// 	// Read recieved value
// 	recieved_value = SPI1->DR;

// 	// Wait until SPI is not busy anymore
// 	//while (SPI1->SR & (SPI_I2S_FLAG_BSY));

// 	// Return received data from SPI data register
// 	return recieved_value;
// }

// void SPI_EnableSlave(void)
// {
// 	// Set slave SS pin low
// 	digitalWrite(PORT_A, 4, 0);
// }

// void SPI_DisableSlave(void)
// {
// 	// Wait until data gets form TX buffer to shift register
// 	while (!(SPI1->SR & (SPI_I2S_FLAG_TXE)));

// 	// Wait until SPI is not busy anymore
// 	while (SPI1->SR & (SPI_I2S_FLAG_BSY));

// 	// Set slave SS pin high
// 	digitalWrite(PORT_A, 4, 1);
// }

// //TIMERS-------------------------------------------------------------------------------------------------
// /*
// void TIMER3_Init_Millisec()
// {
// 	//example code for timer millisec counter

// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

// 	TIM_TimeBaseInitTypeDef timer_base;                    //create struct instance
// 	TIM_TimeBaseStructInit(&timer_base);                   //fill with default values
// 	timer_base.TIM_Prescaler = 72 - 1;                     //fill your value (prescale from 0 to 65535, 1 000 000 Hz)
// 	timer_base.TIM_Period = 1000 - 1;							 //
// 	TIM_TimeBaseInit(TIM3, &timer_base);                   //initialize timer

// 	__disable_irq();													 //disable all interruptions
// 	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);             //tune timers interruptions parametres
// 	NVIC_EnableIRQ(TIM3_IRQn);                             //enable interruptions of specific timer
// 	__enable_irq();													 //enable all interruptions
// 	TIM_Cmd(TIM3, ENABLE);                                 //start count
// }

// void TIM3_IRQHandler(void)
// {
// 	TIM_ClearFlag(TIM3, TIM_IT_Update);							 //reset interruption flag
// 	Millis++;															 //increment every 1 millisec
// }
// */

void systemTimeInit(void)
{
	SysTick_Config(SystemCoreClock / SYSTICKFREQ);   //1 mcs
}

// //function for SysTick timer interruption
void SysTick_Handler(void)
{
	//increments on every SysTick timer interruption
	//see frequency in *SYSTICKFREQ* define
	system_time++;
}
//END OF TIMERS------------------------------------------------------------------------------------------

//UART----------------------------------------------------------------------------------------
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
// void UART1_init(uint32_t baud_rate)
// {
// 	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;						//uart clock
// 	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;						   //port A clock

// 	GPIOA->CRH &= (~GPIO_CRH_CNF9_0);							//PA9 CNF alternate function output open-drain
// 	GPIOA->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9);		//PA9 MODE Output 50 MHz

// 	GPIOA->CRH &= (~GPIO_CRH_CNF10_0);							//PA10 CNF input with pull-up/pull-down
// 	GPIOA->CRH |= GPIO_CRH_CNF10_1;
// 	GPIOA->CRH &= (~(GPIO_CRH_MODE10));							//PA10 MODE input
// 	GPIOA->BSRR |= GPIO_ODR_ODR10;								//ODR = 1

// 	USART1->CR1 = USART_CR1_UE;									//USART1 enable, clear bits
// 	USART1->CR1 |= USART_CR1_M;
// 	USART1->CR1 |= USART_CR1_PCE;

// 	USART1->BRR = 72000000 / baud_rate;							//USARTDIV = Fck / (16 * BAUD) = 72000000 / (16 * 9600) = 468,75 -> USART_BRR = 468,75 * 16 = 7500 (for 9600 baud rate)

// 	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;				//recieving and transmitting enable
// 	USART1->CR2 = 0;
// 	USART1->CR3 = 0;
// }

// void UART1_sendByte(char byte)
// {
// 	//wait for transmitter ready
// 	while ((USART1->SR & USART_SR_TXE) == 0)
// 	{

// 	}

// 	//write byte to register for transmitting
// 	USART1->DR = byte;
// }

// void UART1_sendString(char* string)
// {
// 	while (*string != 0)
// 	{
// 		UART1_sendByte(*string);
// 		string++;
// 	}
// }

// void UART1_print(long data)
// {
// 	if (data < 0)
// 	{
// 		UART1_sendByte('-');
// 		data = -data;
// 	}

// 	UART1_printNumber(data);
// }

// void UART1_print_str(char* string)
// {
// 	UART1_sendString(string);
// }

// void UART1_print_div(double data)
// {
// 	if (isnan(data))
// 	{
// 		return UART1_print_str("nan");
// 	}
// 	else if (isinf(data))
// 	{
// 		return UART1_print_str("inf");
// 	}
// 	else if (data > 4294967040.0)
// 	{
// 		return UART1_print_str("ovf");
// 	}
// 	else if (data < -4294967040.0)
// 	{
// 		return UART1_print_str("-ovf");
// 	}

// 	if (data < 0.0)
// 	{
// 		UART1_sendByte('-');
// 		data = -data;
// 	}

// 	long rounding = 0;
// 	long number_left = data;
// 	uint8_t number_right = 0;

// 	rounding = data * 1000;
// 	uint8_t tmp = rounding % 10;
// 	rounding = rounding / 10;

// 	if (tmp >= 5)
// 	{
// 		number_right = (long)(rounding + 1) % 100;
// 	}
// 	else
// 	{
// 		number_right = (long)rounding % 100;
// 	}

// 	UART1_print(number_left);
// 	UART1_sendByte('.');
// 	UART1_print(number_right);
// }

// void UART1_println(long data)
// {
// 	UART1_print(data);

// 	UART1_sendByte('\n');
// }

// void UART1_println_str(char* string)
// {
// 	UART1_print_str(string);

// 	UART1_sendByte('\n');
// }

// void UART1_println_div(double data)
// {
// 	UART1_print_div(data);

// 	UART1_sendByte('\n');
// }

// void UART1_printNumber(unsigned long number)
// {
// 	char string[11];
// 	char* pointer = &string[sizeof(string) - 1];

// 	*pointer = '\0';
// 	pointer--;

// 	if (number == 0)
// 	{
// 		*(pointer--) = '0';
// 	}
// 	else
// 	{
// 		while (number)
// 		{
// 			char symbol = number % 10;
// 			number = number / 10;

// 			*(pointer--) = symbol + '0';
// 		}
// 	}

// 	return UART1_sendString(pointer + 1);
// }
//END OF UART---------------------------------------------------------------------------------
