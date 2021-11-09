/***********************************************
 * Main file
************************************************/

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"
#include "i2c.h"
#include "adc.h"
#include "splitmind_f401_hal_lib.h"
#include "hal_dynamixel_ax12_a.h"
#include "mpu9250.h"

#define MPU9250_ADDRESS 0xD0
#define WHO_AM_I  0x75

uint8_t data1[2] = { 17, 99 };
uint8_t data2[2] = { 0, 0 };

int16_t servoData[6] = { 0, 0, 0, 0, 0, 0 };
int16_t dummy[6] = { 0, 0, 0, 0, 0, 0 };

uint8_t byte = 0;
uint8_t reg_address = WHO_AM_I;

bool flag = 0;

void setup()
{
   //init all periph with HAL generated functions
   initPeriph();

   led_error(1);
   led_loop(1);

   HAL_Delay(500);
   led_error(0);
   led_loop(0);

   initAllDynamixel();
}

//for button on NUCLEO
void pushButton()
{
   // if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1)
   // {
   //    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
   // }
   // else
   // {
   //    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
   // }
}

//test specific servo with move CW and CCW with desired pause
void testMoveServo(uint8_t servo_id, uint16_t pause)
{
   uint16_t speed = 150;
   wheelMode(servo_id, 1);
   setVelocity(servo_id, speed);
   HAL_Delay(pause);
   setVelocity(servo_id, speed + 1024);
   HAL_Delay(pause);
}

void servoTestAngle(uint8_t servo_id)
{
   int16_t angle = 0;

   pingServo(servo_id);
   jointMode(servo_id);

   setAngle(servo_id, 475);
   HAL_Delay(1000);

   angle = getAngle(servo_id);

   UART_printStr("q: ");
   UART_printDivLn(angle * (float)300 / (float)1024);

   setAngle(servo_id, 525);
   HAL_Delay(1000);

   angle = getAngle(servo_id);

   UART_printStr("q: ");
   UART_printDivLn(angle * (float)300 / (float)1024);
}

void servoTestVel(uint8_t servo_id)
{
   int16_t vel = 0;

   pingServo(servo_id);
   wheelMode(servo_id, 1);

   setVelocity(servo_id, 100);
   HAL_Delay(1000);

   vel = getVelocity(servo_id);

   UART_printStr("vel: ");
   UART_printLn(vel);

   setVelocity(servo_id, 100 + 1024);
   HAL_Delay(1000);

   vel = getVelocity(servo_id);

   UART_printStr("vel: ");
   UART_printLn(vel);
}

void testAngleVel(uint8_t servo_id)
{
   int16_t angle = 0;
   int16_t vel = 0;

   wheelMode(servo_id, 1);

   setVelocity(servo_id, 100);

   vel = getVelocity(servo_id);
   angle = getAngle(servo_id);
   // vel = getVelocity(servo_id);

   UART_printStr("q: ");
   UART_printDivLn(angle * (float)300 / (float)1024);

   UART_printStr("vel: ");
   UART_printDivLn(vel);

   HAL_Delay(500);

   vel = getVelocity(servo_id);
   angle = getAngle(servo_id);
   // vel = getVelocity(servo_id);

   UART_printStr("q: ");
   UART_printDivLn(angle * (float)300 / (float)1024);

   UART_printStr("vel: ");
   UART_printDivLn(vel);

   HAL_Delay(1000);




   setVelocity(servo_id, 100 + 1024);

   vel = getVelocity(servo_id);
   angle = getAngle(servo_id);

   UART_printStr("q: ");
   UART_printDivLn(angle * (float)300 / (float)1024);

   UART_printStr("vel: ");
   UART_printDivLn(vel);

   HAL_Delay(500);

   vel = getVelocity(servo_id);
   angle = getAngle(servo_id);

   UART_printStr("q: ");
   UART_printDivLn(angle * (float)300 / (float)1024);

   UART_printStr("vel: ");
   UART_printDivLn(vel);

   HAL_Delay(1000);
}

//feedback test of desired servo
void servoTest(uint8_t servo_id)
{
   int16_t angle = 0;
   int16_t vel = 0;
   int16_t torque = 0;

   pingServo(servo_id);

   jointMode(servo_id);
   setVelocity(servo_id, 100 + 1024);

   setAngle(servo_id, 300);

   angle = getAngle(servo_id);
   vel = getVelocity(servo_id);
   torque = getTorque(servo_id);

   UART_printStr("q: ");
   UART_printDivLn(angle * (float)300 / (float)1024);

   UART_printStr("vel: ");
   UART_printDivLn(vel);

   UART_printStr("torque: ");
   UART_printDivLn(torque);

   HAL_Delay(1000);

   setAngle(servo_id, 800);

   angle = getAngle(servo_id);
   vel = getVelocity(servo_id);
   torque = getTorque(servo_id);

   UART_printStr("q: ");
   UART_printDivLn(angle * (float)300 / (float)1024);

   UART_printStr("vel: ");
   UART_printDivLn(vel);

   UART_printStr("torque: ");
   UART_printDivLn(torque);

   HAL_Delay(1000);

}

void legTest(uint8_t leg_num)
{
   pingServo(leg_num * 3);
   pingServo(leg_num * 3 + 1);
   pingServo(leg_num * 3 + 2);

   jointMode(leg_num * 3);
   jointMode(leg_num * 3 + 1);
   jointMode(leg_num * 3 + 2);

   setVelocity(leg_num * 3, 100 + 1024);
   setVelocity(leg_num * 3 + 1, 100 + 1024);
   setVelocity(leg_num * 3 + 2, 100 + 1024);

   setAngle(leg_num * 3, 475);
   setAngle(leg_num * 3 + 1, 475);
   setAngle(leg_num * 3 + 2, 475);
   HAL_Delay(1000);

   setAngle(leg_num * 3, 525);
   setAngle(leg_num * 3 + 1, 525);
   setAngle(leg_num * 3 + 2, 525);
   HAL_Delay(1000);
}

void legDataTest(uint8_t leg_num)
{
   int16_t angle = 0;
   int16_t vel = 0;
   int16_t torque = 0;

   UART_printStr("Leg: ");
   UART_printLn(leg_num);

   angle = getAngle(leg_num * 3);
   vel = getVelocity(leg_num * 3);
   torque = getTorque(leg_num * 3);

   UART_printStr("q: ");
   UART_printDiv(angle * (float)300 / (float)1024);
   UART_printStr(" vel: ");
   UART_printDiv(vel);
   UART_printStr(" torque: ");
   UART_printDivLn(torque);

   angle = getAngle(leg_num * 3 + 1);
   vel = getVelocity(leg_num * 3 + 1);
   torque = getTorque(leg_num * 3 + 1);

   UART_printStr(" q: ");
   UART_printDiv(angle * (float)300 / (float)1024);
   UART_printStr(" vel: ");
   UART_printDiv(vel);
   UART_printStr(" torque: ");
   UART_printDivLn(torque);

   angle = getAngle(leg_num * 3 + 2);
   vel = getVelocity(leg_num * 3 + 2);
   torque = getTorque(leg_num * 3 + 2);

   UART_printStr(" q: ");
   UART_printDiv(angle * (float)300 / (float)1024);
   UART_printStr(" vel: ");
   UART_printDiv(vel);
   UART_printStr(" torque: ");
   UART_printDivLn(torque);
   UART_printStrLn(" ");
}

void copyLegMovement()
{
   int16_t angle[3] = { 0, 0, 0 };

   // jointMode(3);
   // jointMode(4);
   // jointMode(5);

   angle[0] = getAngle(0);
   angle[1] = getAngle(1);
   angle[2] = getAngle(2);

   setAngle(3, angle[0]);
   setAngle(4, angle[1]);
   setAngle(5, angle[2]);

   UART_printStr("q0: ");
   UART_printDiv(angle[0] * (float)300 / (float)1024);
   UART_printStr(" q1: ");
   UART_printDiv(angle[1] * (float)300 / (float)1024);
   UART_printStr(" q2: ");
   UART_printDivLn(angle[2] * (float)300 / (float)1024);
}

//for spi
void transferData()
{
   // int16_t angle = 0;
   // int16_t torque = 0;

   HAL_Delay(10);

   // for (uint8_t i = 0; i < 3; i++)
   // {
   //    angle = getAngle(i);
   //    torque = getTorque(i);
   //    servoData[i * 2] = angle;
   //    servoData[i * 2 + 1] = torque;
   // }

   // angle = getAngle(0);
   // torque = getTorque(0);
   // servoData[0] = angle;
   // servoData[1] = torque;

   // angle = getAngle(1);
   // torque = getTorque(1);
   // servoData[2] = angle;
   // servoData[3] = torque;

   // angle = getAngle(2);
   // torque = getTorque(2);
   // servoData[4] = angle;
   // servoData[5] = torque;

   servoData[0] = getAngle(0);
   servoData[1] = getTorque(0);

   servoData[2] = getAngle(1);
   servoData[3] = getTorque(1);

   servoData[4] = getAngle(2);
   servoData[5] = getTorque(2);
}

//recieve spi interrupt
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
   flag = 1;

   //test transfer
   HAL_SPI_TransmitReceive_IT(&hspi1, data1, data2, 2);

   UART_printStr("b1: ");
   UART_print(data2[0]);
   UART_printStr(" b2: ");
   UART_printLn(data2[1]);

   //for visual force of leg
   // HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)servoData, (uint8_t *)dummy, sizeof(servoData));
}

//interruption after tx
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
   //start waiting for recieve interrupt
   HAL_I2C_Master_Receive_IT(&hi2c1, MPU9250_ADDRESS, &byte, 1);
}

//recieve data interrupt
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
   // I2C data ready!
   UART_printLn(byte);
   byte = 0;
}

void measureVbat()
{
   float v_raw = 0;
   float v_bat = 0;
   uint16_t raw;
   float avr_raw = 0;
   uint8_t avr = 25;

   for (size_t i = 0; i < avr; i++)
   {
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

      raw = HAL_ADC_GetValue(&hadc1);
      HAL_Delay(1);
      avr_raw += (float)raw;
   }

   avr_raw = avr_raw / (float)avr;

   v_raw = (float)avr_raw / 4096 * (float)3.3;
   UART_printDiv(v_raw);
   UART_printStr(" ");

   v_bat = v_raw * 7.8f;
   UART_printDivLn(v_bat);
}


int main()
{
   setup();

   // HAL_SPI_Receive_IT(&hspi1, data2, 2);

   // HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)servoData, (uint8_t *)dummy, sizeof(servoData));

   // HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)data1, (uint8_t *)data2, sizeof(data2));

   // setAngle(0, 512);
   // setAngle(1, (150 + 60) * DEG_TO_TICK);
   // setAngle(2, (150 + 110) * DEG_TO_TICK);

   // changeId(UART1, 8);

   HAL_I2C_Master_Transmit_IT(&hi2c1, MPU9250_ADDRESS, &reg_address, 1);

   while (1)
   {
      // measureVbat();

      // HAL_Delay(1000);
      servoTest(6);
      // testAngleVel(6);
      // servoTestAngle(6);
      // servoTestVel(6);

      // pingServo(6);

      // testMoveServo(6, 1000);
      // pingSpecificServo(UART1, 6);

      // transferData();

      // HAL_SPI_Receive(&hspi1, data, 2, HAL_MAX_DELAY);
      // HAL_SPI_Receive_IT(&hspi1, data, 2);

      // if (data2[0] != 0 && data2[1] != 0)
      // if (flag == 1)
      // {
      //    UART_printStr("Byte 0: ");
      //    UART_print(data2[0]);
      //    UART_printStr(" byte 1: ");
      //    UART_printLn(data2[1]);
      //    flag = 0;
      // }

      // UART_printStr("Byte 0: ");
      // UART_print(data[0]);
      // UART_printStr(" byte 1: ");
      // UART_printLn(data[1]);

      // copyLegMovement();
      // legTest(0);
      // legTest(1);

      // legDataTest(0);
      // HAL_Delay(500);
      // legDataTest(1);

      // HAL_Delay(100);
      // pushButton();
      // testMove(500);

      // testMoveServo(s, 500);

      // setAngle(s, 475);
      // HAL_Delay(500);
      // setAngle(s, 525);
      // HAL_Delay(500);

      // setVelocity(s, 150 + 1024);
      // HAL_Delay(500);
      // setVelocity(s, 100 + 1024);
      // HAL_Delay(500);

      // t1 = HAL_GetTick();
      // angle = getAngle(s);
      // vel = getVelocity(s);
      // torque = getTorque(s);
      // TE = getTorqueEnable(1);
      // disableTorque(1);
      // t2 = HAL_GetTick();
      // if (angle == ERROR)
      // {

      // }
      // else
      // {
      //    setAngle(1, (uint16_t)angle);
      // }    

      // UART_printStr("q: ");
      // UART_printDivLn(angle * (float)300 / (float)1024);

      // UART_printStr("vel: ");
      // UART_printDivLn(vel);

      // UART_printStr("torque: ");
      // UART_printDivLn(torque);

      // UART_printStr("TE: ");
      // UART_printDivLn(TE);

      // UART_printStr("t: ");
      // UART_printLn(t2 - t1);

      // UART_printLn(HAL_GetTick());

      led_loop(1);
      HAL_Delay(100);
      led_loop(0);
   }

   return 0;
}
