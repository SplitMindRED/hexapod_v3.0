#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "splitmind_f401_hal_lib.h"
#include "hal_dynamixel_ax12_a.h"

void setup()
{
   initPeriph();

   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void pushButton()
{
   if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1)
   {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
   }
   else
   {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
   }
}

void testMove(uint16_t pause)
{
   wheelMode(1, 1);
   // wheelMode(2, 1);

   setVelocity(1, 100);
   // setVelocity(2, 100);

   HAL_Delay(pause);
   setVelocity(1, 100 + 1024);
   // setVelocity(2, 100 + 1024);
   HAL_Delay(pause);
}

void testMoveServo(uint8_t servo_id, uint16_t pause)
{
   uint16_t speed = 150;
   wheelMode(servo_id, 1);
   setVelocity(servo_id, speed);
   HAL_Delay(pause);
   setVelocity(servo_id, speed + 1024);
   HAL_Delay(pause);
}

void servoTest(uint8_t servo_id)
{
   int16_t angle = 0;
   int16_t vel = 0;
   int16_t torque = 0;

   pingServo(servo_id);

   jointMode(servo_id);
   setVelocity(servo_id, 100 + 1024);

   setAngle(servo_id, 475);
   HAL_Delay(1000);
   setAngle(servo_id, 525);
   HAL_Delay(1000);

   angle = getAngle(servo_id);
   vel = getVelocity(servo_id);
   torque = getTorque(servo_id);

   UART_printStr("q: ");
   UART_printDivLn(angle * (float)300 / (float)1024);

   UART_printStr("vel: ");
   UART_printDivLn(vel);

   UART_printStr("torque: ");
   UART_printDivLn(torque);
}

void legTest(uint8_t leg_num)
{
   pingServo(leg_num);
   pingServo(leg_num + 1);
   pingServo(leg_num + 2);

   jointMode(leg_num);
   jointMode(leg_num + 1);
   jointMode(leg_num + 2);

   setVelocity(leg_num, 100 + 1024);
   setVelocity(leg_num + 1, 100 + 1024);
   setVelocity(leg_num + 2, 100 + 1024);

   setAngle(leg_num, 475);
   setAngle(leg_num + 1, 475);
   setAngle(leg_num + 2, 475);
   HAL_Delay(1000);

   setAngle(leg_num, 525);
   setAngle(leg_num + 1, 525);
   setAngle(leg_num + 2, 525);
   HAL_Delay(1000);
}

int main()
{
   setup();

   int16_t angle = 0;
   int16_t vel = 0;
   int8_t TE = 0;
   int16_t torque = 0;

   unsigned long t1 = 0, t2 = 0;

   // uint8_t s = 2;
   // changeId(s);

   // jointMode(s);
   // setAngle(s, 500);
   // vel = getVelocity(1);

   // wheelMode(s, 1);
   // setVelocity(s, 150 + 1024);

   while (1)
   {
      legTest(0);
      // servoTest(0);
      // servoTest(1);
      // servoTest(2);

      // pingServo(1);
      // pingServo(2);
      // pingServo(s);

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

      // HAL_Delay(50);
   }

   return 0;
}
