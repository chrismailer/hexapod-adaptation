/**
 * @file CommsManager.h
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief CommsManager class
 **/

#ifndef INCLUDE_COMMSMANAGER_H
#define INCLUDE_COMMSMANAGER_H

#include "stm32f4xx.h"
#include "enumDefinitions.h"

/**
 * 	@brief Communications Manager object.
 *
 *	@details Manages all communications initialisations and transmission and reception of data.
 *
 **/
class CommsManager{

private:
	/** @brief HAL library UART1 initialization structure.  **/
	UART_HandleTypeDef UART1_HandleStructure;
	/** @brief HAL library UART2 initialization structure. **/
	UART_HandleTypeDef UART2_HandleStructure;
	/** @brief HAL library UART3 initialization structure. **/
	UART_HandleTypeDef UART3_HandleStructure;

	/** @brief HAL library DMA2 TX (UART1) initialization structure. **/
	DMA_HandleTypeDef DMA2_HandleStructure_UART1_TX;
	/** @brief HAL library  DMA2 RX (UART1) initialization structure. **/
	DMA_HandleTypeDef DMA2_HandleStructure_UART1_RX;

	/** @brief HAL library  DMA1 TX (UART2) initialization structure. **/
	DMA_HandleTypeDef DMA1_HandleStructure_UART2_TX;
	/** @brief HAL library  DMA1 RX (UART2) initialization structure. **/
	DMA_HandleTypeDef DMA1_HandleStructure_UART2_RX;

	/** @brief HAL library  DMA1 TX (UART3) initialization structure. **/
	DMA_HandleTypeDef DMA1_HandleStructure_UART3_RX;
	/** @brief HAL library  DMA1 RX (UART3) initialization structure. **/
	DMA_HandleTypeDef DMA1_HandleStructure_UART3_TX;

public:

	/** @brief Counter to hold the number of failed remote connections. **/
	uint8_t remoteFailureCount = 0;

	/**
	 * @brief Default constructor for the class.
	 *
	 * @details Class constructor.
	 **/
	CommsManager(void);

	/**
	 * @brief Default destructor for the class.
	 *
	 * @details Class destructor cleans up memory by deleting pointers and lists.
	 **/
	~CommsManager(void);

	/**
	 * @brief Initialise the UART modules.
	 *
	 * @details
	 *
	 **/
	void UART_init(void);


	/**
	 * @brief Sets the remote baud rate.
	 *
	 * @details
	 *
	 **/
	void setRemoteBaud(uint32_t BAUD);

	/**
	 * @brief Initialise the DMA modules.
	 *
	 * @details
	 *
	 **/
	void DMA_init(void);

	/**
	 * @brief Get the UART struct for the motors (UART1).
	 *
	 * @details
	 *
	 * @return A pointer to the motor UART struct.
	 **/
	UART_HandleTypeDef * getMotorUART();

	/**
	 * @brief Get the UART struct for the remote (UART2).
	 *
	 * @details
	 *
	 * @return A pointer to the remote UART struct.
	 **/
	UART_HandleTypeDef * getRemoteUART();

	/**
	 * @brief Get the UART struct for the IMU (UART3).
	 *
	 * @details
	 *
	 * @return A pointer to the IMU UART struct.
	 **/
	UART_HandleTypeDef * getAccUART();

	/**
	 * @brief Get the DMA struct for the motors (DMA2 TX/RX).
	 *
	 * @details
	 *
	 * @param tx_rx COMM_DIR enum specifying the DMA struct to return.
	 *
	 * @return A pointer to the motor DMA struct.
	 **/
	DMA_HandleTypeDef * getMotorDMA(COMM_DIR tx_rx);

	/**
	 * @brief Get the DMA struct for the remote (DMA1 TX/RX).
	 *
	 * @details
	 *
	 * @param tx_rx COMM_DIR enum specifying the DMA struct to return.
	 *
	 * @return A pointer to the remote DMA struct.
	 **/
	DMA_HandleTypeDef * getRemoteDMA(COMM_DIR tx_rx);

	/**
	 * @brief Get the DMA struct for the IMU (DMA1 TX/RX).
	 *
	 * @details
	 *
	 * @param tx_rx COMM_DIR enum specifying the DMA struct to return.
	 *
	 * @return A pointer to the IMU DMA struct.
	 **/
	DMA_HandleTypeDef * getAccDMA(COMM_DIR tx_rx);

	/**
	 * @brief Transmit data to the motors.
	 *
	 * @details
	 *
	 * @param data Pointer to the data array to transmit.
	 * @param size Size of the data array to transmit.
	 * @param timeout Timeout for blocking transmissions.
	 * @param type Type of transmission (DMA, INTERRUPT, BLOCKING).
	 *
	 * @return True or false as success or failure of transmission.
	 **/
	HAL_StatusTypeDef transmitMotors(uint8_t * data, uint8_t size, uint16_t timeout, enum UART_TRANSMISSION_TYPE type);

	/**
	 * @brief Transmit data to the remote.
	 *
	 * @details
	 *
	 * @param data Pointer to the data array to transmit.
	 * @param size Size of the data array to transmit.
	 * @param timeout Timeout for blocking transmissions.
	 * @param type Type of transmission (DMA, INTERRUPT, BLOCKING).
	 *
	 * @return True or false as success or failure of transmission.
	 **/
	HAL_StatusTypeDef transmitRemote(uint8_t * data, uint8_t size, uint16_t timeout, enum UART_TRANSMISSION_TYPE type);

	/**
	 * @brief Transmit data to the IMU.
	 *
	 * @details
	 *
	 * @param data Pointer to the data array to transmit.
	 * @param size Size of the data array to transmit.
	 * @param timeout Timeout for blocking transmissions.
	 * @param type Type of transmission (DMA, INTERRUPT, BLOCKING).
	 *
	 * @return True or false as success or failure of transmission.
	 **/
	HAL_StatusTypeDef transmitAcc(uint8_t * data, uint8_t size, uint16_t timeout, enum UART_TRANSMISSION_TYPE type);

	/**
	 * @brief Receive data from the motors.
	 *
	 * @details
	 *
	 * @param data Pointer to the data array to receive.
	 * @param size Size of the data array to receive.
	 * @param timeout Timeout for blocking reception.
	 * @param type Type of reception (DMA, INTERRUPT, BLOCKING).
	 *
	 * @return True or false as success or failure of reception.
	 **/
	HAL_StatusTypeDef receiveMotors(uint8_t * data, uint8_t size, uint16_t timeout, enum UART_TRANSMISSION_TYPE type);

	/**
	 * @brief Receive data from the remote.
	 *
	 * @details
	 *
	 * @param data Pointer to the data array to receive.
	 * @param size Size of the data array to receive.
	 * @param timeout Timeout for blocking reception.
	 * @param type Type of reception (DMA, INTERRUPT, BLOCKING).
	 *
	 * @return True or false as success or failure of reception.
	 **/
	HAL_StatusTypeDef receiveRemote(uint8_t * data, uint8_t size, uint16_t timeout, enum UART_TRANSMISSION_TYPE type);

	/**
	 * @brief Receive data from the IMU.
	 *
	 * @details
	 *
	 * @param data Pointer to the data array to receive.
	 * @param size Size of the data array to receive.
	 * @param timeout Timeout for blocking reception.
	 * @param type Type of reception (DMA, INTERRUPT, BLOCKING).
	 *
	 * @return True or false as success or failure of reception.
	 **/
	HAL_StatusTypeDef receiveAcc(uint8_t * data, uint8_t size, uint16_t timeout, enum UART_TRANSMISSION_TYPE type);


};

#endif

