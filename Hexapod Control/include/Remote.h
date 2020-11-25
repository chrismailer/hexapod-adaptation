/**
 * @file Remote.h
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief  Remote class
 **/

#ifndef INCLUDE_REMOTE_H
#define INCLUDE_REMOTE_H



#include "stm32f4xx.h"
#include "Diag/Trace.h"

#include "enumDefinitions.h"

#include "CommsManager.h"

/**
 * 	@brief Class for sending data to and receiving data from the remote control.
 *
 *	@details
 *
 **/
class Remote{

private:
	/** @brief A pointer to the global communications manager object. **/
	CommsManager * comms;

	/** @brief An array to hold the incoming remote data. **/
	uint8_t dma_dataPacket[10];

	/** @brief An array to hold the IMU data to transmit. **/
	uint8_t dma_acc_transmit[26];

	/** @brief An array to hold the error data to transmit. **/
	uint8_t dma_error_transmit[4];

public:
	/** @brief A boolean trigger that will be true when new data is available. **/
	bool newData = false;
	/** @brief The speed data received from the remote. **/
	double speed = 0;
	/** @brief The direction data received from the remote. **/
	double direction = 0;
	/** @brief The body height data received from the remote. **/
	double bodyHeight = 0;
	/** @brief The button function data received from the remote. **/
	uint8_t function = 0;
	/** @brief The twist angle of the body. **/
	double twistBody = 0;

	/** @brief A state variable to determine if the remote is currently sending data. **/
	bool sending = false;

	/** @brief Boolean holding the status of the last ping. **/
	bool pingSuccess = false;

	/** @brief Boolean holding the current charging status. **/
	bool charging = false;

	/**
	 * @brief Default constructor for the class.
	 *
	 * @details Class constructor initializing the accelerometer remote_uart pointer.
	 *
	 * @param comms A pointer to the global communications manager object.
	 *
	 **/
	Remote(CommsManager * comms);

	/**
	 * @brief Default destructor for the class.
	 *
	 * @details Class destructor cleans up memory by deleting pointers and lists.
	 *
	 **/
	~Remote(void);

	/**
	 * @brief Send an ASCII string through the remote.
	 *
	 * @details
	 *
	 * @param msg A pointer to the char array to send.
	 *
	 **/
	void sendASCII(char * msg);

	/**
	 * @brief Send the start or stop charging command to the charging base.
	 *
	 * @details
	 *
	 * @param charging True or false to start or stop charging respectively.
	 *
	 **/
	void sendCharging(bool charging);

	/**
	 * 	@brief Send the IMU data through the remote.
	 *
	 *	@details
	 *	@param pitch The pitch of the IMU.
	 *	@param roll The roll of the IMU.
	 *	@param yaw The yaw of the IMU.
	 *	@param timestamp The timestamp of the IMU.
	 *	@param slopePitch The slope pitch of the Hexapod.
	 *	@param slopeRoll The slope roll of the Hexapod.
	 **/
	bool sendAcc(double pitch, double roll, double yaw, uint16_t timestamp, double slopePitch, double slopeRoll);

	/**
	 * 	@brief Send error data through the remote.
	 *
	 *	@details
	 *
	 *	@param errorLoc The location of the error (UART1, UART2, UART3)
	 *	@param errorCode The error code (Frame error, parity error etc...)
	 *
	 **/
	void sendError(uint8_t errorLoc, uint8_t errorCode);

	/**
	 * 	@brief Receive the remote data using DMA.
	 *
	 *	@details
	 *
	 **/
	void receiveRemoteData(uint8_t size);

	/**
	 * 	@brief Check if the checksum sent by the remote control is valid.
	 *
	 *	@details
	 *
	 *	@return True or false whether or not the checksum is valid.
	 **/
	bool isChecksumValid(void);

	/**
	 * 	@brief Process the remote data.
	 *
	 *	@details
	 *
	 **/
	void processRemoteData(void);

	/**
	 * 	@brief Convert a float into a byte array
	 *
	 *	@details
	 *
	 *	@param value The float to convert.
	 *	@param bytes A pointer to the array to populate with the bytes.
	 *
	 **/
	void floatToBytes(float value, uint8_t * bytes);

	/**
	 * 	@brief Convert a byte array into a float.
	 *
	 *	@details
	 *
	 *	@param bytes The byte array to convert.
	 *
	 **/
	float bytesToFloat(uint8_t * bytes);

	/**
	 * 	@brief Return the pingSuccess boolean.
	 *
	 *	@details
	 *
	 *
	 **/
	bool successfulPing(void);

};

#endif
