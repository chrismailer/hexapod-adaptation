/**
 * @file Motor.h
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief Motor class
 **/
#ifndef INCLUDE_MOTOR_H
#define INCLUDE_MOTOR_H

#include "stm32f4xx.h"
#include "enumDefinitions.h"
#include "diag/Trace.h"

#include "CommsManager.h"
/**
 * 	@brief Class for controlling an individual motor.
 *
 *	@details
 *
 **/
class Motor{

private:
	/** @brief A pointer to the global communications manager. **/
	CommsManager * comms;

	/** @brief  Array holding the data bits for the goal position registers. **/
	uint8_t goalMovementBits[4] = {0,0,0,0};

	/** @brief Array holding the data bits for the present position registers. **/
	uint8_t presentMovementBits[4] = {0,0,0,0};

	/** @brief The size of data to be read from the control table. **/
	uint8_t readDataSize = 0;


public:
	/** @brief Array holding the return data from the motor ping command. **/
	uint8_t pingStatusPacket[6] = {0,0,0,0,0,0};
	/** @brief Array holding the data from the motors control table. **/
	uint8_t controlTable[56] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	/** @brief Array holding the status packet from a motor read command. **/
	uint8_t dma_motor_statusPacket[7];
	/** @brief Constant holding the ID of the motor. **/
	const uint8_t ID;
	/** @brief Allowable error between goal position and present position. **/
	uint8_t error = 0;
	/** @brief A packet populated with the transmission data required to read from the motor.   **/
	uint8_t readPacket[8] = {0,0,0,0,0,0,0,0};

	/**
	 * @brief Default constructor for the class.
	 *
	 * @details Class constructor initializing the motor ID.
	 *
	 * @param IDVal The motor ID
	 * @param comms A point to the CommsManager object.
	 *
	 **/
	Motor(uint8_t IDVal, CommsManager * comms);

	/**
	 * @brief Default destructor for the class.
	 *
	 * @details Class destructor cleans up memory by deleting pointers and lists.
	 *
	 **/
	~Motor(void);

	/**
	 * 	@brief Send the 'ping' command to the motor.
	 *
	 *	@details
	 *
	 *	@return boolean
	 **/
	bool pingBool(void);

	/**
	 * 	@brief Send the 'ping' command to the motor.
	 *
	 *	@details
	 *
	 *	@return boolean
	 **/
	uint8_t * pingData(void);

	/**
	 * 	@brief Reset any errors on the motor.
	 *
	 *	@details
	 **/
	void resetErrors(void);

	/**
	 * 	@brief Clear the motor control table (only in this object not in the motor itself).
	 *
	 *	@details
	 *
	 **/
	void clearControlTable();

	/**
	 * 	@brief	Calculates the data bytes for a goal theta position.
	 *
	 *	@details
	 *
	 *	@param theta The theta position in radians
	 *
	 **/
	void calcPositionBits(double theta);

	/**
	 * 	@brief Calculates the data byte for a goal speed.
	 *
	 *	@details
	 *
	 *	@param theta_dot The speed in revolutions per minute (RPM).
	 *
	 **/
	void calcSpeedBits(double theta_dot);

	/**
	 * 	@brief	Sets the goal position and speed of the motor.
	 *
	 *	@details
	 *
	 *	@param theta The goal position in radians.
	 *	@param speed The goal speed in RPM.
	 *
	 **/
	void setGoalPositionSpeed(double theta, double speed);	//Radians and RPM

	/**
	 * 	@brief Returns the goal position bytes array.
	 *
	 *	@details
	 *
	 *	@return A pointer to goalMovementBits array.
	 **/
	uint8_t * getGoalMovementBits(void);

	/**
	 * 	@brief Returns the present position bytes array.
	 *
	 *	@details
	 *
	 *	@return A pointer to presentMovementBits array.
	 **/
	uint8_t * getPresentMovementBits(void);

	/**
	 * 	@brief	Sets the position bytes array of the motor.
	 *
	 *	@param pos The motor position as a number from 0 to 1023 (0 to 300 degrees).
	 *
	 *	@details
	 *
	 **/
	void setPositionBits(uint16_t pos);

	/**
	 * 	@brief	Sets the speed bytes array of the motor.
	 *
	 * 	@param speed The motor speed as a number from 0 to 1023 (0 to 114 RPM).
	 *
	 *	@details
	 *
	 **/
	void setSpeedBits(uint16_t speed);

	/**
	 * 	@brief	Sends the command to move the motor to the currently set goal position.
	 *
	 *	@details
	 *
	 **/
	void move(void);

	/**
	 * 	@brief	Writes data to one or more registers.
	 *
	 *	@param reg The register to start writing to.
	 *	@param data A pointer to the data to write.
	 *	@param dataLength The length of the data to be written.
	 *
	 *	@details
	 *
	 **/
	void writeReg(enum CONTROL_TABLE reg, uint8_t * data, uint8_t dataLength);

	/**
	 * 	@brief	Writes one register value to either 1 or 0 (ENABLE or DISABLE).
	 *
	 *	@param reg The register to write to.
	 *	@param state The value to write.
	 *
	 *	@details
	 *
	 **/
	void writeReg(enum CONTROL_TABLE reg, FunctionalState state);

	/**
	 * 	@brief	Reads one or more register values from the motor.
	 *
	 *	@param reg The register to begin reading from.
	 *	@param dataLength The length of data to read.
	 *
	 *	@details
	 *
	 *	@return The success/failure state of the read.
	 *
	 **/
	bool readReg(enum CONTROL_TABLE reg, uint8_t dataLength);

	/**
	 * 	@brief	Reads all register values from the motor.
	 *
	 *	@details
	 *
	 *	@return The success/failure state of the read.
	 **/
	bool readTable();

};

#endif
