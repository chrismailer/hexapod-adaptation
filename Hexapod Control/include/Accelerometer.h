/**
 * @file Accelerometer.h
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief Accelerometer.h class
 **/

#ifndef INCLUDE_ACCELEROMETER_H
#define INCLUDE_ACCELEROMETER_H


#include "stm32f4xx.h"
#include "diag/Trace.h"
#include "Math.h"

#include "enumDefinitions.h"

#include "CommsManager.h"

/**
 * 	@brief Class for controlling and reading data from the hexapod accelerometer via UART.
 *
 *	@details
 *
 **/
class Accelerometer{

private:
	/** @brief Pointer to global communications manager object.  **/
	CommsManager * comms;

	/** @brief Data to send the device to measurement mode.  **/
	uint8_t gotoMeasurement[5] = {0xFA, 0xFF, 0x10, 0x00, 0xF1};
	/** @brief Data to send the device to configuration mode.  **/
	uint8_t gotoConfig[5] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};

	/** @brief Data to send the device to reset the heading (yaw) to zero.  **/
	uint8_t resetHeadingData[7] = {0xFA, 0xFF, 0xA4, 0x02, 0x00, 0x01, 0x5A};

	/** @brief Data to send to request data from the device.  **/
	uint8_t reqData[5] = {0xFA,0x01,0x34,0x00,0xCB};

	/** @brief Trigger whether the transmission is data or configuration. **/
	enum ACCEL_TRANSMISSIONS accel_transmission = DATA;

	/** brief dma_infoPacket Packet holding returned info from device configuration reading.  **/
	uint8_t dma_infoPacket[5] = {0,0,0,0,0};
	/** @brief Packet holding returned data from device measurement reading. **/
	uint8_t dma_dataPacket[55] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
								  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	/**
	 * @brief Converts bytes from the @ref dma_dataPacket into a float.
	 *
	 * @details Given a start index in the @ref dma_dataPacket class object the next 4 bytes are converted into a float and returned.
	 *
	 * @param startIndex The starting index in the array to begin converting from.
	 *
	 * @return The float made up from 4 bytes.
	 **/
	float bytesToFloat(int startIndex);

	/**
	 * 	@brief Check if the checksum sent by the accelerometer is valid.
	 *
	 *	@details
	 *
	 * 	@return True or false whether or not the checksum is valid.
	 **/
	bool isChecksumValid(void);

public:

	/** @brief Roll angle of the device.  **/
	float roll = 0;
	/** @brief Pitch angle of the device. **/
	float pitch = 0;
	/** @brief Yaw angle of the device.  **/
	float yaw = 0;

	/** @brief Timestamp of the device.  **/
	uint16_t timestamp = 0;

	/**
	 * @brief Default constructor for the class.
	 *
	 * @details Class constructor initializing the accelerometer accel_uart pointer.
	 *
	 * @param comms Pointer to the communications manager object
	 **/
	Accelerometer(CommsManager * comms);

	/**
	 * @brief Default destructor for the class.
	 *
	 * @details Class destructor cleans up memory by deleting pointers and lists.
	 **/
	~Accelerometer(void);

	/**
	 * 	@brief Request data from the accelerometer.
	 *
	 *	@details Send an instruction to the accelerometer using DMA which request the accelerometer to return its data.
	 **/
	void requestAccelData(void);

	/**
	 * 	@brief Receive data from the accelerometer using DMA.
	 *
	 *	@details Receive data returned from the accelerometer. The data is either configuration data or measurement data based on the value
	 *	of accel_transmission.
	 *
	 **/
	void receiveAccelData(void);

	/**
	 * 	@brief Process the accelerometer data once it has been received.
	 *
	 *	@details Process the incoming accelerometer data, pulling out the various bits of data and combining them into pitch, roll and yaw.
	 *	Only if the checksum is correct meaning that the data has not been corrupted.
	 *
	 **/
	void processAccelData(void);

	/**
	 * 	@brief Set the device to measurement mode.
	 *
	 *	@details Once the device is set to measurement mode, the requestAccelData function can be used.
	 *
	 **/
	void setMeasurementMode(void);

	/**
	 * 	@brief Set the device to configuration mode.
	 *
	 * 	@details The device must be set to configuration mode before sending any commands to change device settings.
	 *
	 **/
	void setConfigMode(void);

	/**
	 * 	@brief
	 *
	 * 	@details
	 *
	 **/
	void resetHeading(bool confirm);

};

#endif
