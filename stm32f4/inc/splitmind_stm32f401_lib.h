/***********************************************
*	SplitMind Library for STM32F401RE
*	Version 0.1
*
*  Functions for low level initialization and control
*
*	WRITING BITs
*	a |= 1 << 7;					//set 1 in 7th bit
*	a &= ~(1 << 3);				//set 0 in 3th bit
*	a ^= 1 << 5;					//inversion of 5th bit
*	a |= 1 << 7 | 1 << 8		   //sets 1 to 7th and 8th bits
*
*	READING BITs
*	if( a & (1<<7) )				//if 7th bit in "a" var equals 1 -> true
*  a & ( 1 << 7 | 1 << 8 )	   //checks 7th and 8th bits
*
************************************************/

#ifndef SPLITMIND_STM32F401_LIB_h
#define SPLITMIND_STM32F401_LIB_h

#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "stdbool.h"
#include "math.h"
#include "stdint.h"

#define SYSTICKFREQ                 1000000

// //DEFINE PORT LETTER
// #define PORT_A				            (2)
// #define PORT_B                      (3)
// #define PORT_C                      (4)
// #define PORT_D                      (5)
// #define PORT_E                      (6)
// #define PORT_F                      (7)
// #define PORT_G                      (8)

// //DEFINE MODES OF PINS
// #define INPUT							   (0)
// #define OUTPUT_2						   (2)
// #define OUTPUT_10						   (1)
// #define OUTPUT_50						   (3)

// //DEFINE CONFIG OF MODES FOR INPUT
// #define INPUT_ANALOG					   (0)
// #define INPUT_FLOAT						(1)
// #define INPUT_PULL_UP_DOWN				(2)

// //DEFINE CONFIG OF MODES FOR OUTPUT
// #define OUTPUT_GPO_PUSH_PULL			(0)
// #define OUTPUT_GPO_OPEN_DRAIN		   (1)
// #define OUTPUT_AF_PUSH_PULL			(2)
// #define OUTPUT_AF_OPEN_DRAIN			(3)

extern volatile unsigned long system_time;

typedef struct SoftTimer_ms
{
   unsigned long delay;
   unsigned long start_time;
} SoftTimer_ms;

//--------------------------------------------

// void pinMode(uint8_t port, uint8_t pin, uint8_t mode, uint8_t config);
void digitalWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool value);
// bool digitalRead(uint8_t port, uint8_t pin);

float map(float have, float have_min, float have_max, float need_min, float need_max);

bool checkTimer(SoftTimer_ms* timer);

//change core frequency to 84 MHz
void changeCoreFrequency(void);

//hard delay in milliseconds, empty cycle
bool delayms(uint32_t millisec);

//hard delay in microseconds, empty cycle
bool delayus(uint32_t microsec);

//I2C------------------------------------------------------------------------------------------
// void I2C1_init(void);
// void I2C_writeByte(uint8_t device_address, uint8_t address, uint8_t data);
// void I2C_burstWrite(uint8_t device_address, uint8_t address, uint8_t n_data, uint8_t* data);
// uint8_t I2C_readByte(uint8_t device_address, uint8_t address);
// void I2C_burstRead(uint8_t device_address, uint8_t address, uint8_t n_data, uint8_t* data);
//END OF I2C-----------------------------------------------------------------------------------

//SPI------------------------------------------------------------------------------------------
// void SPI_init(void);
// uint8_t SPI_Transfer(uint8_t data);
// void SPI_EnableSlave(void);
// void SPI_DisableSlave(void);
// uint16_t eval_CRC(void);
//END OF SPI-----------------------------------------------------------------------------------

//TIMERS---------------------------------------------------------------------------------------
//void TIMER3_Init_Millisec();
/* for ppm pin A7 takes TIM3_CH2*/
//function for SysTick timer interruption
void systemTimeInit(void);
void SysTick_Handler(void);
//END OF TIMERS-------------------------------------------------------------------------------

//UART----------------------------------------------------------------------------------------
void usartInit2(void);
void usartHalfDuplexInit6(void);  //USART 6
void usartHalfDuplexInit1(void);  //USART 1
void sendByteln(char c);
void sendByte(uint8_t byte);
void sendByteArray(uint8_t* p, uint8_t length);
void sendByte1(uint8_t byte);
void sendByteArray1(uint8_t* p, uint8_t length);

// void UART1_init(uint32_t baud_rate);
// void UART1_sendByte(char byte);
// void UART1_sendString(char* string);
// void UART1_print(long data);
// void UART1_print_str(char* string);
// void UART1_print_div(double data);
// void UART1_println(long data);
// void UART1_println_str(char* string);
// void UART1_println_div(double data);
// void UART1_printNumber(unsigned long number);
//END OF UART---------------------------------------------------------------------------------

#endif
