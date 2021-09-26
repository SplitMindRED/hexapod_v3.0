/***********************************************
*	Dynamixel AX12-A
*	Version 0.1
*  0xFF 0xFF ID LENGTH INSTRUCTION PARAM_1 ... PARAM_N CHKSUM
*  CHKSUM = ~(ID + LENGTH + INSTRUCTION + PARAM_1 + … PARAM_N)
 * Status Checksum = ~( ID + Length + Error + Parameter1 + … Parameter N )
*  LENGTH = NUMBER_OF_INSTRUCTIONS + 2
************************************************/

#ifndef DYNAMIXEL_AX12_A_H
#define DYNAMIXEL_AX12_A_H

#include "main.h"
#include "splitmind_f401_hal_lib.h"

#define U1_DEBUG

#define TICK_TO_DEG (float)300 / (float)1024
#define DEG_TO_TICK (float)1024 / (float)300

#define REC_BUFFER_LEN            32
#define SERVO_MAX_PARAMS          (REC_BUFFER_LEN - 5)

#define REC_WAIT_START_US         75
#define REC_WAIT_PARAMS_US        (SERVO_MAX_PARAMS * 5)
#define REC_WAIT_MAX_RETRIES      200

#define SERVO_INSTRUCTION_ERROR   (1 << 6)
#define SERVO_OVERLOAD_ERROR      (1 << 5)
#define SERVO_CHECKSUM_ERROR      (1 << 4)
#define SERVO_RANGE_ERROR         (1 << 3)
#define SERVO_OVERHEAT_ERROR      (1 << 2)
#define SERVO_ANGLE_LIMIT_ERROR   (1 << 1)
#define SERVO_INPUT_VOLTAGE_ERROR (1)

#define RETURN_DELAY              0x05
#define BLINK_CONDITIONS          0x11
#define SHUTDOWN_CONDITIONS       0x12
#define TORQUE                    0x22
#define MAX_SPEED                 0x20
#define CURRENT_SPEED             0x26
#define GOAL_ANGLE                0x1e
#define CURRENT_ANGLE             0x24

// Instruction Set ///////////////////////////////////////////////////////////////
#define AX_PING                     1
#define AX_READ_DATA                2
#define AX_WRITE_DATA               3
#define AX_REG_WRITE                4
#define AX_ACTION                   5
#define AX_RESET                    6
#define AX_SYNC_WRITE               131

// EEPROM AREA  ///////////////////////////////////////////////////////////
#define AX_MODEL_NUMBER_L           0
#define AX_MODEL_NUMBER_H           1
#define AX_VERSION                  2
#define AX_ID                       3
#define AX_BAUD_RATE                4
#define AX_RETURN_DELAY_TIME        5
#define AX_CW_ANGLE_LIMIT_L         6
#define AX_CW_ANGLE_LIMIT_H         7
#define AX_CCW_ANGLE_LIMIT_L        8
#define AX_CCW_ANGLE_LIMIT_H        9
#define AX_SYSTEM_DATA2             10
#define AX_LIMIT_TEMPERATURE        11
#define AX_DOWN_LIMIT_VOLTAGE       12
#define AX_UP_LIMIT_VOLTAGE         13
#define AX_MAX_TORQUE_L             14
#define AX_MAX_TORQUE_H             15
#define AX_RETURN_LEVEL             16
#define AX_ALARM_LED                17
#define AX_ALARM_SHUTDOWN           18
#define AX_OPERATING_MODE           19
#define AX_DOWN_CALIBRATION_L       20
#define AX_DOWN_CALIBRATION_H       21
#define AX_UP_CALIBRATION_L         22
#define AX_UP_CALIBRATION_H         23

// RAM AREA  //////////////////////////////////////////////////////////////
#define AX_TORQUE_ENABLE            24
#define AX_LED                      25
#define AX_CW_COMPLIANCE_MARGIN     26
#define AX_CCW_COMPLIANCE_MARGIN    27
#define AX_CW_COMPLIANCE_SLOPE      28
#define AX_CCW_COMPLIANCE_SLOPE     29
#define AX_GOAL_POSITION_L          30
#define AX_GOAL_POSITION_H          31
#define AX_GOAL_SPEED_L             32
#define AX_GOAL_SPEED_H             33
#define AX_TORQUE_LIMIT_L           34
#define AX_TORQUE_LIMIT_H           35
#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_POSITION_H       37
#define AX_PRESENT_SPEED_L          38
#define AX_PRESENT_SPEED_H          39
#define AX_PRESENT_LOAD_L           40
#define AX_PRESENT_LOAD_H           41
#define AX_PRESENT_VOLTAGE          42
#define AX_PRESENT_TEMPERATURE      43
#define AX_REGISTERED_INSTRUCTION   44
#define AX_PAUSE_TIME               45
#define AX_MOVING                   46
#define AX_LOCK                     47
#define AX_PUNCH_L                  48
#define AX_PUNCH_H                  49

// Status Return Levels ///////////////////////////////////////////////////////////////
#define AX_RETURN_NONE              0
#define AX_RETURN_READ              1
#define AX_RETURN_ALL               2

// Specials ///////////////////////////////////////////////////////////////
#define OFF                         0
#define ON                          1
#define LEFT						      0
#define RIGHT                       1
#define AX_BYTE_READ                1
#define AX_BYTE_READ_POS            2
#define AX_PING_LENGTH				   2
#define AX_RESET_LENGTH				   2
#define AX_ACTION_LENGTH			   2
#define AX_ID_LENGTH                4
#define AX_LR_LENGTH                4
#define AX_SRL_LENGTH               4
#define AX_RDT_LENGTH               4
#define AX_LEDALARM_LENGTH          4
#define AX_SALARM_LENGTH            4
#define AX_TL_LENGTH                4
#define AX_VL_LENGTH                6
#define AX_CM_LENGTH                6
#define AX_CS_LENGTH                6
#define AX_CCW_CW_LENGTH            8
#define AX_BD_LENGTH                4
#define AX_TEM_LENGTH               4
#define AX_MOVING_LENGTH            4
#define AX_RWS_LENGTH               4
#define AX_VOLT_LENGTH              4
#define AX_LED_LENGTH               4
#define AX_TORQUE_LENGTH            4
#define AX_POS_LENGTH               4
#define AX_GOAL_LENGTH              5
#define AX_MT_LENGTH                5
#define AX_PUNCH_LENGTH             5
#define AX_SPEED_LENGTH             5
#define AX_GOAL_SP_LENGTH           7
#define AX_ACTION_CHECKSUM			   250
#define BROADCAST_ID                254
#define AX_START                    255
#define AX_CCW_AL_L                 255 
#define AX_CCW_AL_H                 3
#define TIME_OUT                    10
#define TX_MODE                     1
#define RX_MODE                     0
#define LOCK                        1

extern unsigned long delta;

extern uint8_t servoErrorCode;
extern bool flag;

typedef struct ServoResponse
{
   uint8_t id;
   uint8_t length;
   uint8_t error;
   uint8_t params[SERVO_MAX_PARAMS];
   uint8_t checksum;
   int8_t result;
} ServoResponse;

extern volatile uint8_t receiveBuffer[REC_BUFFER_LEN];
extern volatile uint8_t *volatile receiveBufferStart;
extern volatile uint8_t *volatile receiveBufferEnd;

typedef enum ServoCommand
{
   PING = 1,
   READ = 2,
   WRITE = 3
} ServoCommand;

// ping a servo, returns true if we get back the expected values
int8_t pingServo(uint8_t servo_id);
int8_t changeId(uint8_t new_id);

void jointMode(uint8_t servo_id);
void wheelMode(uint8_t servo_id, bool status);

int16_t getAngle(uint8_t servo_id);
void getVelocity(uint8_t servo_id);
void getTorque(uint8_t servo_id);

void setAngle(uint8_t servo_id, uint16_t angle);
void setVelocity(uint8_t servo_id, int16_t velocity);
void setTorque(uint8_t servo_id, int16_t torque);

ServoResponse checkResponse(uint8_t servo_id, uint8_t *p_answer);

#endif
