/***********************************************
*	Dynamixel AX12-A
************************************************/

#include "hal_dynamixel_ax12_a.h"

Servo servo[18];

void led_loop(bool flag)
{
   if (flag == 1)
   {
      HAL_GPIO_WritePin(LOOP_LED, GPIO_PIN_SET);
   }
   else
   {
      HAL_GPIO_WritePin(LOOP_LED, GPIO_PIN_RESET);
   }
}

void led_error(bool flag)
{
   if (flag == 1)
   {
      HAL_GPIO_WritePin(ERROR_LED, GPIO_PIN_SET);
   }
   else
   {
      HAL_GPIO_WritePin(ERROR_LED, GPIO_PIN_RESET);
   }
}

// void led_board(bool flag)
// {
//    if (flag == 1)
//    {
//       HAL_GPIO_WritePin(ONBOARD_LED, GPIO_PIN_SET);
//    }
//    else
//    {
//       HAL_GPIO_WritePin(ONBOARD_LED, GPIO_PIN_RESET);
//    }
// }

int8_t initAllDynamixel(void)
{
   for (size_t i = 0; i < 18; i++)
   {
      servo[i].mode = 0;
      servo[i].velocity = 0;
      servo[i].angle = 0;
      servo[i].torque = 0;
      servo[i].is_moving = false;
   }


   return OK;
}

uint8_t HAL_Transmit(uint8_t servo_id, uint8_t *packet, uint8_t size)
{
   uint8_t hal_return = 0;

   if (servo_id == 3 || servo_id == 4 || servo_id == 5 || servo_id == 6 || servo_id == 7 || servo_id == 8)
   {
      hal_return = HAL_UART_Transmit(UART1, packet, size, MAX_DELAY);
   }
   else if (servo_id == 0 || servo_id == 1 || servo_id == 2 || servo_id == 9 || servo_id == 10 || servo_id == 11)
   {
      hal_return = HAL_UART_Transmit(UART2, packet, size, MAX_DELAY);
   }
   else if (servo_id == 12 || servo_id == 13 || servo_id == 14 || servo_id == 15 || servo_id == 16 || servo_id == 17)
   {
      hal_return = HAL_UART_Transmit(UART6, packet, size, MAX_DELAY);
   }

   return hal_return;
}

uint8_t HAL_Recieve(uint8_t servo_id, uint8_t *answer, uint8_t size)
{
   uint8_t hal_return = 0;

   if (servo_id == 3 || servo_id == 4 || servo_id == 5 || servo_id == 6 || servo_id == 7 || servo_id == 8)
   {
      hal_return = HAL_UART_Receive(UART1, answer, size, MAX_DELAY);
   }
   else if (servo_id == 0 || servo_id == 1 || servo_id == 2 || servo_id == 9 || servo_id == 10 || servo_id == 11)
   {
      hal_return = HAL_UART_Receive(UART2, answer, size, MAX_DELAY);
   }
   else if (servo_id == 12 || servo_id == 13 || servo_id == 14 || servo_id == 15 || servo_id == 16 || servo_id == 17)
   {
      hal_return = HAL_UART_Receive(UART6, answer, size, MAX_DELAY);
   }

   return hal_return;
}

int8_t pingSpecificServo(UART_HandleTypeDef *huart, uint8_t servo_id)
{
   unsigned char packet[6];
   unsigned char checksum;
   uint8_t hal_return = 0;

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


   HAL_UART_Transmit(huart, packet, sizeof(packet), MAX_DELAY);

   hal_return = HAL_UART_Receive(huart, answer, 7, MAX_DELAY);

   if (hal_return != HAL_OK)
   {

      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" recieve ping FAIL!");

      led_error(1);

      return ERROR;
   }

   ServoResponse response = checkResponse(servo_id, answer);

   if (response.result == OK)
   {

      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" ping SUCCSESS!");

      led_error(0);

      return OK;
   }
   else
   {

      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" ping FAIL!");

      led_error(1);

      return ERROR;
   }
}

int8_t pingServo(uint8_t servo_id)
{
   unsigned char packet[6];
   unsigned char checksum;
   uint8_t hal_return = 0;

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

   HAL_Transmit(servo_id, packet, sizeof(packet));

   hal_return = HAL_Recieve(servo_id, answer, 7);

   if (hal_return != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" recieve ping FAIL!");
#endif
      led_error(1);
      return ERROR;
   }

   ServoResponse response = checkResponse(servo_id, answer);

   if (response.result == OK)
   {
#ifdef U_DEBUG
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" ping SUCCSESS!");
#endif
      led_error(0);
      return OK;
   }
   else
   {
#ifdef U_DEBUG
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" ping FAIL!");
#endif      
      led_error(1);
      return ERROR;
   }
}

int8_t jointMode(uint8_t servo_id)
{
   unsigned char packet[9];
   uint8_t limit_L = 1023;
   uint8_t limit_H = 1023 >> 8;
   unsigned char checksum = 0;
   uint8_t hal_return = 0;

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

   hal_return = HAL_Transmit(servo_id, packet, sizeof(packet));

   if (hal_return == HAL_OK)
   {
      servo[servo_id].mode = JOINT_MODE;

      led_error(0);

#ifdef U_DEBUG
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" in joint mode!");
#endif      

      return OK;
   }
   else
   {
      led_error(1);

#ifdef U_DEBUG
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" set joint mode TRANSMIT FAIL!");
#endif     

      return ERROR;
   }
}

int8_t wheelMode(uint8_t servo_id)
{
   unsigned char packet[9];
   unsigned char checksum;
   uint8_t hal_return = 0;

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

   hal_return = HAL_Transmit(servo_id, packet, sizeof(packet));

   if (hal_return == HAL_OK)
   {
      servo[servo_id].mode = WHEEL_MODE;

      led_error(0);

#ifdef U_DEBUG
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" in wheel mode!");
#endif      

      return OK;
   }
   else
   {
      led_error(1);

#ifdef U_DEBUG
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" set wheel mode TRANSMIT FAIL!");
#endif     

      return ERROR;
   }
}

int16_t getAngle(uint8_t servo_id)
{
   unsigned char packet[8];
   unsigned char checksum = 0;
   uint8_t hal_return = 0;

   uint8_t answer[20];

   for (uint8_t i = 0; i < 20; i++)
   {
      answer[i] = 0;
   }

   checksum = (~(servo_id + AX_POS_LENGTH + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_2_BYTE_READ));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = servo_id;
   packet[3] = AX_POS_LENGTH;
   packet[4] = AX_READ_DATA;
   packet[5] = AX_PRESENT_POSITION_L;
   packet[6] = AX_2_BYTE_READ;
   packet[7] = checksum; //CD

   hal_return = HAL_Transmit(servo_id, packet, sizeof(packet));

   if (hal_return != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStrLn("Read pos transmit FAIL!");
#endif
      led_error(1);

      return ERROR;
   }

   hal_return = HAL_Recieve(servo_id, answer, 9);

   if (hal_return != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStrLn("Read pos recieve answer FAIL!");
#endif
      led_error(1);

      return ERROR;
   }

   ServoResponse response = checkResponse(servo_id, answer);

   if (response.result == OK)
   {
      uint16_t pos = response.params[0] | (response.params[1] << 8);

      servo[servo_id].angle = pos;

      led_error(0);

      return pos;
   }
   else
   {
#ifdef U_DEBUG
      UART_printStrLn("Read pos FAIL!");
#endif

      led_error(1);

      return ERROR;
   }
}

int16_t getVelocity(uint8_t servo_id)
{
   unsigned char packet[8];
   unsigned char checksum = 0;
   uint8_t hal_return = 0;

   uint8_t answer[20];

   for (uint8_t i = 0; i < 20; i++)
   {
      answer[i] = 0;
   }

   checksum = (~(servo_id + AX_SPEED_LENGTH_R + AX_READ_DATA + AX_PRESENT_SPEED_L + AX_2_BYTE_READ));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = servo_id;
   packet[3] = AX_SPEED_LENGTH_R;
   packet[4] = AX_READ_DATA;
   packet[5] = AX_PRESENT_SPEED_L;
   packet[6] = AX_2_BYTE_READ;
   packet[7] = checksum;

   //6 4 2 26 ... CB

   // hal_return = HAL_Transmit(servo_id, packet, sizeof(packet));

   if (servo_id == 3 || servo_id == 4 || servo_id == 5 || servo_id == 6 || servo_id == 7 || servo_id == 8)
   {
      hal_return = HAL_UART_Transmit(UART1, packet, sizeof(packet), MAX_DELAY);
   }
   else if (servo_id == 0 || servo_id == 1 || servo_id == 2 || servo_id == 9 || servo_id == 10 || servo_id == 11)
   {
      hal_return = HAL_UART_Transmit(UART2, packet, sizeof(packet), MAX_DELAY);
   }
   else if (servo_id == 12 || servo_id == 13 || servo_id == 14 || servo_id == 15 || servo_id == 16 || servo_id == 17)
   {
      hal_return = HAL_UART_Transmit(UART6, packet, sizeof(packet), MAX_DELAY);
   }

   if (hal_return != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStrLn("Read vel transmit FAIL!");
#endif
      led_error(1);

      return ERROR;
   }

   hal_return = HAL_Recieve(servo_id, answer, 9);

   if (hal_return != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStr("Read vel recieve answer FAIL! ");
      UART_printLn(hal_return);
#endif
      led_error(1);

      return ERROR;
   }

   ServoResponse response = checkResponse(servo_id, answer);

   if (response.result == OK)
   {
      int16_t vel = response.params[0] | (response.params[1] << 8);

      servo[servo_id].velocity = vel;

      // if ((vel & 1 << 10))
      // {
      //    vel &= ~(1 << 10);
      //    vel = -vel;
      // }

      led_error(0);

      return vel;
   }
   else
   {
#ifdef U_DEBUG
      UART_printStr("Servo ");
      UART_print(servo_id);
      UART_printStrLn(" read velocity FAIL!");
#endif

      led_error(1);

      return ERROR;
   }
}

int16_t getTorque(uint8_t servo_id)
{
   unsigned char packet[8];
   unsigned char checksum = 0;
   uint8_t hal_return = 0;

   uint8_t answer[20];

   for (uint8_t i = 0; i < 20; i++)
   {
      answer[i] = 0;
   }

   checksum = (~(servo_id + AX_TORQUE_LENGTH_R + AX_READ_DATA + AX_PRESENT_LOAD_L + AX_2_BYTE_READ));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = servo_id;
   packet[3] = AX_TORQUE_LENGTH_R;
   packet[4] = AX_READ_DATA;
   packet[5] = AX_PRESENT_LOAD_L;
   packet[6] = AX_2_BYTE_READ;
   packet[7] = checksum;

   hal_return = HAL_Transmit(servo_id, packet, sizeof(packet));

   if (hal_return != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStrLn("Read torque transmit FAIL!");
#endif
      led_error(1);

      return ERROR;
   }

   hal_return = HAL_Recieve(servo_id, answer, 9);

   if (hal_return != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStrLn("Read torque recieve answer FAIL!");
#endif
      led_error(1);

      return ERROR;
   }

   ServoResponse response = checkResponse(servo_id, answer);

   if (response.result == OK)
   {
      int16_t torque = response.params[0] | (response.params[1] << 8);

      servo[servo_id].torque = torque;

      // if ((torque & 1 << 10))
      // {
      //    torque &= ~(1 << 10);
      //    torque = -torque;
      // }

      led_error(0);

      return torque;
   }
   else
   {
#ifdef U_DEBUG
      UART_printStrLn("Read torque FAIL!");
#endif

      led_error(1);

      return ERROR;
   }
}

int8_t setAngle(uint8_t servo_id, uint16_t angle)
{
   unsigned char packet[9];
   uint8_t angle_L = angle;
   uint8_t angle_H = angle >> 8;
   unsigned char checksum = 0;
   uint8_t hal_return = 0;

   checksum = (~(servo_id + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + angle_L + angle_H));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = servo_id;
   packet[3] = AX_GOAL_LENGTH;
   packet[4] = AX_WRITE_DATA;
   packet[5] = AX_GOAL_POSITION_L;     //1E
   packet[6] = angle_L;
   packet[7] = angle_H;
   packet[8] = checksum;

   hal_return = HAL_Transmit(servo_id, packet, sizeof(packet));

   if (hal_return != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStrLn("Set angle transmit FAIL!");
#endif
      led_error(1);

      return ERROR;
   }

   uint8_t answer[20];

   hal_return = HAL_Recieve(servo_id, answer, 7);

   if (hal_return != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStrLn("Set angle recieve answer FAIL!");
#endif
      led_error(1);

      return ERROR;
   }

   ServoResponse response = checkResponse(servo_id, answer);

   if (response.result == OK)
   {
      led_error(0);

      return OK;
   }
   else
   {
#ifdef U_DEBUG
      UART_printStrLn("Set angle response FAIL!");
#endif

      led_error(1);

      return ERROR;
   }
}

int8_t setVelocity(uint8_t servo_id, int16_t velocity)
{
   uint8_t velocity_L, velocity_H;
   velocity_H = velocity >> 8;
   velocity_L = velocity;                     // 16 bits - 2 x 8 bits variables
   unsigned char checksum;
   uint8_t hal_return = 0;

   unsigned char packet[9];

   checksum = (~(servo_id + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + velocity_L + velocity_H));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = servo_id;
   packet[3] = AX_SPEED_LENGTH;
   packet[4] = AX_WRITE_DATA;
   packet[5] = AX_GOAL_SPEED_L;// 0x20
   packet[6] = velocity_L;
   packet[7] = velocity_H;
   packet[8] = checksum; //6D

   // packet[0] = 0xFF;
   // packet[1] = 0xFF;
   // packet[2] = 0x01;
   // packet[3] = 0x05;
   // packet[4] = 0x03;
   // packet[5] = 0x20;
   // packet[6] = 0x64;
   // packet[7] = 0x00;
   // packet[8] = 0x72;

   hal_return = HAL_Transmit(servo_id, packet, sizeof(packet));

   if (hal_return != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStrLn("Set velocity transmit FAIL!");
#endif
      led_error(1);

      return ERROR;
   }

   uint8_t answer[20];

   hal_return = HAL_Recieve(servo_id, answer, 7);

   if (hal_return != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStrLn("Set vel recieve answer FAIL!");
#endif
      led_error(1);

      return ERROR;
   }

   ServoResponse response = checkResponse(servo_id, answer);

   if (response.result == OK)
   {
      led_error(0);

      return OK;
   }
   else
   {
#ifdef U_DEBUG
      UART_printStrLn("Set vel response FAIL!");
#endif

      led_error(1);

      return ERROR;
   }
}

int8_t setTorqueLimit(uint8_t servo_id, int16_t torque)
{

}

int8_t getTorqueEnable(uint8_t servo_id)
{
   unsigned char packet[8];
   unsigned char checksum = 0;

   uint8_t answer[20];

   for (uint8_t i = 0; i < 20; i++)
   {
      answer[i] = 0;
   }

   checksum = (~(servo_id + 4 + AX_READ_DATA + AX_TORQUE_ENABLE + AX_BYTE_READ));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = servo_id;
   packet[3] = 4;
   packet[4] = AX_READ_DATA;
   packet[5] = AX_TORQUE_ENABLE;
   packet[6] = AX_BYTE_READ;
   packet[7] = checksum;

   if (HAL_UART_Transmit(UART1, packet, sizeof(packet), MAX_DELAY) != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStrLn("Read torque enable transmit FAIL!");
#endif
      turnLed(1);

      // return ERROR;
   }

   if (HAL_UART_Receive(UART1, answer, 9, MAX_DELAY) != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStrLn("Read torque enable recieve answer FAIL!");
#endif
      turnLed(1);

      // return ERROR;
   }

   ServoResponse response = checkResponse(servo_id, answer);

   if (response.result == OK)
   {
      int8_t torque_enable = response.params[0];
      return torque_enable;
   }
   else
   {
#ifdef U_DEBUG
      UART_printStrLn("Read torque enable FAIL!");
#endif
      return ERROR;
   }
}

int8_t disableTorque(uint8_t servo_id)
{
   unsigned char packet[8];
   unsigned char checksum = 0;

   uint8_t answer[20];

   for (uint8_t i = 0; i < 20; i++)
   {
      answer[i] = 0;
   }

   checksum = (~(servo_id + 4 + AX_WRITE_DATA + AX_TORQUE_ENABLE));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = servo_id;
   packet[3] = 4;
   packet[4] = AX_WRITE_DATA;
   packet[5] = AX_TORQUE_ENABLE;
   packet[6] = 0;
   packet[7] = checksum;

   if (HAL_UART_Transmit(UART1, packet, sizeof(packet), MAX_DELAY) != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStrLn("Torque disable transmit FAIL!");
#endif
      turnLed(1);

      // return ERROR;
   }

   if (HAL_UART_Receive(UART1, answer, 9, MAX_DELAY) != HAL_OK)
   {
#ifdef U_DEBUG
      UART_printStrLn("Torque disable recieve answer FAIL!");
#endif
      turnLed(1);

      // return ERROR;
   }

   ServoResponse response = checkResponse(servo_id, answer);

   if (response.result == OK)
   {
      // int8_t torque_enable = response.params[0];
      return OK;
   }
   else
   {
#ifdef U_DEBUG
      UART_printStrLn("Torque disable FAIL!");
#endif
      return ERROR;
   }
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

int8_t changeId(UART_HandleTypeDef *huart, uint8_t new_id)
{
   unsigned char packet[8];
   unsigned char checksum = 0;

   checksum = (~(BROADCAST_ID + AX_ID_LENGTH + AX_WRITE_DATA + AX_ID + new_id));

   packet[0] = AX_START;
   packet[1] = AX_START;
   packet[2] = BROADCAST_ID;
   packet[3] = AX_ID_LENGTH;
   packet[4] = AX_WRITE_DATA;
   packet[5] = AX_ID;
   packet[6] = new_id;
   packet[7] = checksum;

   HAL_UART_Transmit(huart, packet, sizeof(packet), MAX_DELAY);

   HAL_Delay(100);

   if (pingSpecificServo(huart, new_id))
   {
      led_error(0);

      return OK;
   }
   else
   {
      led_error(1);

      return ERROR;
   }
}
