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
#define WHO_AM_I        0x75
#define INA219_ADDRESS  0x82
#define CONFIGURATION   0x00
#define CURRENT         0x04
#define SHUNT_VOLTAGE   0x01

uint8_t config[2];
uint8_t reg = CONFIGURATION;
uint16_t configuration = 0;
int16_t current = 0;
bool is_new_data = false;

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

   // led_error(1);
   // led_loop(1);

   // HAL_Delay(500);
   // led_error(0);
   // led_loop(0);

   // initAllDynamixel();
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

bool checkKeyButton()
{
   if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1)
   {
      //button is NOT pressed

      // UART_printLn(1);
      return 0;
   }
   else
   {
      HAL_Delay(300);
      //button is pressed

      // UART_printLn(0);
      return 1;
   }
}


//test specific servo with move CW and CCW with desired pause
void testMoveServo(uint8_t servo_id, uint16_t pause)
{
   uint16_t speed = 150;
   wheelMode(servo_id);
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
   wheelMode(servo_id);

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
   int16_t vt = 100;

   wheelMode(servo_id);

   setVelocity(servo_id, vt);

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


   setVelocity(servo_id, vt + 1024);

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
   uint16_t angle = 0;
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
   // HAL_I2C_Master_Receive_IT(&hi2c1, MPU9250_ADDRESS, &byte, 1);
   HAL_I2C_Master_Receive_IT(&hi2c1, INA219_ADDRESS, config, 2);
}

//recieve data interrupt
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
   // I2C data ready!
   // UART_printStr("MPU WHO AM I: ");
   // UART_printLn(byte);
   // byte = 0;

   current = config[0] << 8 | config[1];

   is_new_data = true;
}

void measureVbat()
{
   float v_raw = 0;
   float v_bat = 0;
   uint16_t raw;
   float avr_raw = 0;
   uint8_t avr = 25;

   ADC_Select_CH9();

   for (size_t i = 0; i < avr; i++)
   {
      HAL_ADC_Start(&hadc1);

      HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

      raw = HAL_ADC_GetValue(&hadc1);
      // UART_printDiv(raw);
      // UART_printStr(" ");

      HAL_Delay(1);
      avr_raw += (float)raw;
   }

   HAL_ADC_Stop(&hadc1);

   avr_raw = avr_raw / (float)avr;
   // UART_printDiv(avr_raw);
   // UART_printStr(" ");


   v_raw = (float)avr_raw / 4096 * (float)3.3;
   UART_printDiv(v_raw);
   UART_printStr(" ");

   v_bat = v_raw * 7.8f;
   UART_printDivLn(v_bat);
}

void measureCurrent()
{
   uint16_t raw;
   float v_raw = 0;
   uint8_t avr = 25;
   float avr_raw = 0;
   float current = 0;

   ADC_Select_CH8();

   for (size_t i = 0; i < avr; i++)
   {
      HAL_ADC_Start(&hadc1);

      HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

      raw = HAL_ADC_GetValue(&hadc1);
      // UART_printDiv(raw);
      // UART_printStr(" ");

      HAL_Delay(1);
      avr_raw += (float)raw;
   }

   HAL_ADC_Stop(&hadc1);

   avr_raw = avr_raw / (float)avr;
   // UART_printDiv(avr_raw);
   // UART_printStr(" ");

   v_raw = (float)avr_raw / (float)4096 * (float)3.3;
   UART_printDiv(v_raw);
   UART_printStr(" ");

   current = (v_raw - 2.5) / 0.1;
   UART_printDivLn(current);
}

void readCurrent(void)
{
   reg = SHUNT_VOLTAGE;
   HAL_I2C_Master_Transmit_IT(&hi2c1, INA219_ADDRESS, &reg, 1);
}

void printData(void)
{
   if (is_new_data)
   {
      UART_printStr(" Current: ");

      UART_print(-current / 10);
      UART_printStrLn(" mA");

      // UART_printLn(configuration);

      is_new_data = false;
   }
}

void dynamixelTest(void)
{
   readCurrent();

   // changeId(UART1, 3);
   // servoTest(3);
   // setTorqueLimit(3, 300);

   // jointMode(3);
   // wheelMode(3);

   uint16_t q = 99;
   int16_t dq = 99;
   int16_t torque = 99;
   torque = getTorque(3);
   q = getAngle(3);
   dq = getVelocity(3);

   if (is_new_data)
   {

      UART_printStr("q: ");
      UART_print(q);
      UART_printStr(" dq: ");
      UART_print(dq);
      UART_printStr(" t: ");
      UART_print(torque);

      printData();

   }


   // setAngle(3, 512);
   setVelocity(3, 300 + 1024);

   // impedanceControl(3, 0, 1.5, 500, 100);
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

   // HAL_I2C_Master_Transmit_IT(&hi2c1, MPU9250_ADDRESS, &reg_address, 1);

   // wheelMode(6);
   // jointMode(6);
   // jointMode(8);

   // setVelocity(6, 50);

   // setVelocity(6, 100 + 1024);
   // setVelocity(6, 100); //present load ~40 inwheel mode

   // setVelocity(6, 100 + 1024);
   // setVelocity(6, 200); //~100

   // setVelocity(6, 100 + 1024);
   // setVelocity(6, 1023); //~600

   // setAngle(6, 512);


   // setAngle(8, 0);

   // jointMode(6);

   // setVelocity(6, 100);
   // setAngle(6, 512);

   // uint8_t s = 15;

   // changeId(UART1, s);

   // int16_t torque = 0;
   // int16_t present_speed = 0;
   // int16_t present_pos = 0;
   // float present_angle = 0.0;
   // float present_speed_angle = 0.0;
   // int16_t prev_pos = 0;
   // float calc_vel = 0;
   // float calc_vel_angle = 0.0;

   // unsigned long last_time = 0;
   // unsigned long prev_time = 0;
   // bool flag = 1;
   // bool direction = 0;

   // uint16_t speed = 100;
   // uint16_t dt = 2;


   // changeId(UART1, 6);

   // while (1)
   // {
   //    // pingSpecificServo(UART1, s);

   //    if (checkKeyButton())
   //    {
   //       flag = !flag;
   //    }

   //    if (flag == 0)
   //    {
   //       setVelocity(6, 0);
   //    }
   //    else
   //    {
   //       torque = getTorque(6);
   //       present_speed = getVelocity(6);
   //       present_pos = getAngle(6);
   //       present_angle = (float)present_pos * 300.0 / 1024.0;

   //       torque = torque & 0x7FF;

   //       if ((torque & 1 << 10))
   //       {
   //          torque &= ~(1 << 10);
   //          torque = -torque;
   //       }

   //       present_speed = present_speed & 0x7FF;

   //       if ((present_speed & 1 << 10))
   //       {
   //          present_speed &= ~(1 << 10);
   //          present_speed = -present_speed;
   //       }


   //       // if (HAL_GetTick() >= (last_time + dt * 1000))
   //       // {
   //       //    last_time = HAL_GetTick();

   //       //    flag = !flag;

   //       //    if (flag)
   //       //    {
   //       //       setVelocity(6, speed);
   //       //    }
   //       //    else
   //       //    {
   //       //       setVelocity(6, speed + 1024);
   //       //    }
   //       // }

   //       uint64_t DT;
   //       DT = HAL_GetTick() - prev_time;
   //       // UART_printStr("dt: ");
   //       // UART_print(DT);

   //       calc_vel = (float)(present_pos - prev_pos) * 1000.0 / (float)DT;
   //       calc_vel_angle = (calc_vel * 300.0 / 1024.0);
   //       present_speed_angle = present_speed * 0.674;

   //       UART_printStr(" M: ");
   //       UART_print(torque);
   //       // UART_printStr(" PS: ");
   //       // UART_print(present_speed);
   //       UART_printStr(" PSA: ");
   //       UART_printDiv(present_speed_angle);
   //       // UART_printStr(" pos: ");
   //       // UART_print(present_pos);
   //       UART_printStr(" ang: ");
   //       UART_printDiv(present_angle);
   //       // UART_printStr(" clc vel: ");
   //       // UART_printDiv(calc_vel);
   //       UART_printStr(" clc va: ");
   //       UART_printDivLn(calc_vel_angle);

   //       if (present_pos > 900)
   //       {
   //          direction = 0;
   //       }
   //       else if (present_pos < 123)
   //       {
   //          direction = 1;
   //       }

   //       if (direction == 1)
   //       {
   //          setVelocity(6, speed * 2);
   //       }
   //       else
   //       {
   //          // setVelocity(6, speed + 1024);
   //       }


   //    }

   //    prev_time = HAL_GetTick();
   //    prev_pos = present_pos;

   //    led_loop(1);
   //    HAL_Delay(100);
   //    led_loop(0);
   // }

   while (1)
   {
      dynamixelTest();
   }

   return 0;
}
