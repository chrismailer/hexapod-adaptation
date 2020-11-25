/**
 * @file enumDefinitions.h
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief File containing all of the necessary definitions for the hexapod.
 **/

#ifndef INCLUDE_ENUMDEFINITIONS_H
#define INCLUDE_ENUMDEFINITIONS_H

#include "stm32f4xx.h"

/** @brief The possible charging statuses and base communications.  **/
enum CHARGER_STATUS{
	NOT_CHARGING,
	CHARGING_CC,
	CHARGING_CV,
	FAULT,
	DOCKED,
	START_CHARGE,
	STOP_CHARGE,
	INTERRUPTED,
	LOW_BATTERY,
	RESEND
};

/** @brief A struct holding the battery voltage and charging status.  **/
struct CHARGER{
	enum CHARGER_STATUS status;
	double voltage;
};


/** @brief The accelerometer data transmission type.  **/
enum ACCEL_TRANSMISSIONS{
	CONFIG,
	DATA
};

/** @brief The mode of turning.  **/
enum TURN_MODE{
	DEPENDENT,
	ABSOLUTE
};

/** @brief The UART data transmission type.  **/
enum UART_TRANSMISSION_TYPE{
	BLOCKING,
	INTERRUPT,
	DMA
};

/** @brief The possible motor errors that a motor can return in a status packet. **/
enum MOTOR_ERROR{
	NO_ERROR = 				0b10000000,
	INSTRUCTION_ERROR = 	0b01000000,
	OVERLOAD_ERROR = 		0b00100000,
	CHECKSUM_ERROR = 		0b00010000,
	RANGE_ERROR = 			0b00001000,
	OVERHEATING_ERROR = 	0b00000100,
	ANGLE_LIMIT_ERROR = 	0b00000010,
	INPUT_VOLTAGE_ERROR = 	0b00000001
};

/** @brief The possible UART write type. **/
enum WRITE_TYPE{
	INTERRUPT_DMA,
	STANDARD
};

/** @brief The possible UART communications directions.  **/
enum COMM_DIR{
	TX, RX
};

/** @brief Definition of UART 1 **/
#define UART1 USART1
/** @brief Definition of UART 2 **/
#define UART2 USART2
/** @brief Definition of UART 3 **/
#define UART3 USART3

/** @brief The possible instructions that a motor may be sent. **/
enum INSTRUCTION{
	PING = 0x01, READ_DATA = 0x02, WRITE_DATA = 0x03, REG_WRITE = 0x04, ACTION = 0x05, SYNC_WRITE = 0x83
};

/** @brief The register values of the control table of the dynamixel motors. **/
enum CONTROL_TABLE{
	MODEL_NUMBER_L, // 0x00
	MODEL_NUMBER_H, // 0x01
	VERSION, // 0x02
	ID, // 0x03
	BAUD_RATE, // 0x04
	RETURN_DELAY_TIME, // 0x05
	CW_ANGLE_LIMIT_L, // 0x06
	CW_ANGLE_LIMIT_H, // 0x07
	CCW_ANGLE_LIMIT_L, // 0x08
	CCW_ANGLE_LIMIT_H, // 0x09
	RESERVED1, // 0x0A
	HIGHEST_LIMIT_TEMPERATURE, // 0x0B
	LOWEST_LIMIT_VOLTAGE, // 0x0C
	HIGHEST_LIMIT_VOLTAGE, // 0x0D
	MAX_TORQUE_L, // 0x0E
	MAX_TORQUE_H, // 0x0F
	STATUS_RETURN_LEVEL, // 0x10
	ALARM_LED, // 0x11
	ALARM_SHUTDOWN, // 0x12
	RESERVED2, // 0x13
	RESERVED3, // 0x14
	RESERVED4, // 0x15
	RESERVED5, // 0x16
	RESERVED6, // 0x17
	TORQUE_ENABLE, // 0x18
	LED, // 0x19
	CW_COMPLIANCE_MARGIN, // 0x1A
	CCW_COMPLIANCE_MARGIN, // 0x1B
	CW_COMPLIANCE_SLOPE, // 0x1C
	CCW_COMPLIANCE_SLOPE, // 0x1D
	GOAL_POSITION_L, // 0x1E
	GOAL_POSITION_H, // 0x1F
	MOVING_SPEED_L, // 0x20
	MOVING_SPEED_H, // 0x21
	TORQUE_LIMIT_L, // 0x22
	TORQUE_LIMIT_H, // 0x23
	PRESENT_POSITION_L, // 0x24
	PRESENT_POSITION_H, // 0x25
	PRESENT_SPEED_L, // 0x26
	PRESENT_SPEED_H, // 0x27
	PRESENT_LOAD_L, // 0x28
	PRESENT_LOAD_H, // 0x29
	PRESENT_VOLTAGE, // 0x2A
	PRESENT_TEMPERATURE, // 0x2B
	REGISTERED_INSTRUCTION, // 0x2C
	RESERVE3, // 0x2D
	MOVING, // 0x2E
	LOCK, // 0x2F
	PUNCH_L, // 0x30
	PUNCH_H // 0x31
};

/** @brief The UART received data stream array positions of the control table of the dynamixel motors. **/
enum CONTROL_TABLE_READ_WITH_START_STOP{
	CONTROL_START_BIT_1,				//0
	CONTROL_START_BIT_2,				//1
	CONTROL_START_ID,					//2
	CONTROL_START_LENGTH,				//3
	CONTROL_START_ERROR,				//4

	CONTROL_MODEL_NUMBER_L, 			//5
	CONTROL_MODEL_NUMBER_H, 			//6
	CONTROL_VERSION, 					//7
	CONTROL_ID, 						//8
	CONTROL_BAUD_RATE, 					//9
	CONTROL_RETURN_DELAY_TIME, 			//10
	CONTROL_CW_ANGLE_LIMIT_L, 			//11
	CONTROL_CW_ANGLE_LIMIT_H, 			//12
	CONTROL_CCW_ANGLE_LIMIT_L, 			//13
	CONTROL_CCW_ANGLE_LIMIT_H, 			//14
	CONTROL_RESERVED1, 					//15
	CONTROL_HIGHEST_LIMIT_TEMPERATURE, 	//16
	CONTROL_LOWEST_LIMIT_VOLTAGE, 		//17
	CONTROL_HIGHEST_LIMIT_VOLTAGE, 		//18
	CONTROL_MAX_TORQUE_L, 				//19
	CONTROL_MAX_TORQUE_H, 				//20
	CONTROL_STATUS_RETURN_LEVEL, 		//21
	CONTROL_ALARM_LED, 					//22
	CONTROL_ALARM_SHUTDOWN,	 			//23
	CONTROL_RESERVED2, 					//24
	CONTROL_RESERVED3, 					//25
	CONTROL_RESERVED4, 					//26
	CONTROL_RESERVED5, 					//27
	CONTROL_RESERVED6, 					//28
	CONTROL_TORQUE_ENABLE, 				//29
	CONTROL_LED, 						//30
	CONTROL_CW_COMPLIANCE_MARGIN, 		//31
	CONTROL_CCW_COMPLIANCE_MARGIN, 		//32
	CONTROL_CW_COMPLIANCE_SLOPE, 		//33
	CONTROL_CCW_COMPLIANCE_SLOPE, 		//34
	CONTROL_GOAL_POSITION_L, 			//35
	CONTROL_GOAL_POSITION_H, 			//36
	CONTROL_MOVING_SPEED_L, 			//37
	CONTROL_MOVING_SPEED_H, 			//38
	CONTROL_TORQUE_LIMIT_L, 			//39
	CONTROL_TORQUE_LIMIT_H, 			//40
	CONTROL_PRESENT_POSITION_L, 		//41
	CONTROL_PRESENT_POSITION_H, 		//42
	CONTROL_PRESENT_SPEED_L, 			//43
	CONTROL_PRESENT_SPEED_H, 			//44
	CONTROL_PRESENT_LOAD_L, 			//45
	CONTROL_PRESENT_LOAD_H, 			//46
	CONTROL_PRESENT_VOLTAGE, 			//47
	CONTROL_PRESENT_TEMPERATURE, 		//48
	CONTROL_REGISTERED_INSTRUCTION, 	//49
	CONTROL_RESERVED7, 					//50
	CONTROL_MOVING, 					//51
	CONTROL_LOCK, 						//52
	CONTROL_PUNCH_L, 					//53
	CONTROL_PUNCH_H, 					//54

	CONTROL_END_CHECKSUM				//55
};

/** @brief The rotation values of the different legs of the hexapod.  **/
enum LEGS{
	RELATIVE = -1,
	LEG1 = 0,
	LEG2 = 60,
	LEG3 = 120,
	LEG4 = 180,
	LEG5 = 240,
	LEG6 = 300
};

/** @brief The possible types of a Point object.  **/
enum MOVEMENT{
	XYZ,
	THETA
};

#endif
