/**
 * @file CommsManager.cpp
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief CommsManager class functions.
 **/

#include "CommsManager.h"

// Public Functions

CommsManager::CommsManager(void){
	UNUSED(UART1_HandleStructure);
	UNUSED(UART2_HandleStructure);
	UNUSED(UART3_HandleStructure);

	UNUSED(DMA2_HandleStructure_UART1_TX);
	UNUSED(DMA2_HandleStructure_UART1_RX);

	UNUSED(DMA1_HandleStructure_UART2_TX);
	UNUSED(DMA1_HandleStructure_UART2_RX);

	UNUSED(DMA1_HandleStructure_UART3_RX);
	UNUSED(DMA1_HandleStructure_UART3_TX);
}

CommsManager::~CommsManager(void){

}

void CommsManager::setRemoteBaud(uint32_t BAUD){
	HAL_UART_DeInit(&UART2_HandleStructure);
	UART2_HandleStructure.Init.BaudRate = BAUD;
	HAL_UART_Init (&UART2_HandleStructure);
}

void CommsManager::UART_init(void) {
	__USART1_CLK_ENABLE();
	__USART2_CLK_ENABLE();
	__USART3_CLK_ENABLE();

	UART1_HandleStructure.Init.WordLength = UART_WORDLENGTH_8B;			//UART1
	UART1_HandleStructure.Init.StopBits = UART_STOPBITS_1;
	UART1_HandleStructure.Init.Parity = UART_PARITY_NONE;
	UART1_HandleStructure.Init.Mode = UART_MODE_TX_RX;
	UART1_HandleStructure.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UART1_HandleStructure.Init.OverSampling = UART_OVERSAMPLING_16;
	UART1_HandleStructure.Instance = UART1;
	UART1_HandleStructure.Init.BaudRate = 1000000;
	HAL_UART_Init (&UART1_HandleStructure);

	HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(USART1_IRQn);

	UART2_HandleStructure.Init.WordLength = UART_WORDLENGTH_8B;			//UART2
	UART2_HandleStructure.Init.StopBits = UART_STOPBITS_1;
	UART2_HandleStructure.Init.Parity = UART_PARITY_NONE;
	UART2_HandleStructure.Init.Mode = UART_MODE_TX_RX;
	UART2_HandleStructure.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UART2_HandleStructure.Init.OverSampling = UART_OVERSAMPLING_16;
	UART2_HandleStructure.Instance = UART2;			//changed from USART2 to UART2
	UART2_HandleStructure.Init.BaudRate = 19200;
	HAL_UART_Init (&UART2_HandleStructure);

	HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

	UART3_HandleStructure.Init.WordLength = UART_WORDLENGTH_8B;			//UART3
	UART3_HandleStructure.Init.StopBits = UART_STOPBITS_1;
	UART3_HandleStructure.Init.Parity = UART_PARITY_NONE;
	UART3_HandleStructure.Init.Mode = UART_MODE_TX_RX;
	UART3_HandleStructure.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UART3_HandleStructure.Init.OverSampling = UART_OVERSAMPLING_16;
	UART3_HandleStructure.Instance = UART3;
	UART3_HandleStructure.Init.BaudRate = 115200;
	HAL_UART_Init (&UART3_HandleStructure);

	HAL_NVIC_SetPriority(USART3_IRQn, 1, 2);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void CommsManager::DMA_init(void) {
	__DMA1_CLK_ENABLE();
	__DMA2_CLK_ENABLE();

	/* Peripheral DMA init*/										//UART1 TX
	DMA2_HandleStructure_UART1_TX.Instance = DMA2_Stream7;
	DMA2_HandleStructure_UART1_TX.Init.Channel = DMA_CHANNEL_4;
	DMA2_HandleStructure_UART1_TX.Init.Direction = DMA_MEMORY_TO_PERIPH;
	DMA2_HandleStructure_UART1_TX.Init.PeriphInc = DMA_PINC_DISABLE;
	DMA2_HandleStructure_UART1_TX.Init.MemInc = DMA_MINC_ENABLE;
	DMA2_HandleStructure_UART1_TX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	DMA2_HandleStructure_UART1_TX.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	DMA2_HandleStructure_UART1_TX.Init.Mode = DMA_NORMAL;
	DMA2_HandleStructure_UART1_TX.Init.Priority = DMA_PRIORITY_HIGH;
	DMA2_HandleStructure_UART1_TX.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init (&DMA2_HandleStructure_UART1_TX);
	__HAL_LINKDMA(&UART1_HandleStructure, hdmatx, DMA2_HandleStructure_UART1_TX);
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 4);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

	/* Peripheral DMA init*/										//UART1 RX
	DMA2_HandleStructure_UART1_RX.Instance = DMA2_Stream2;
	DMA2_HandleStructure_UART1_RX.Init.Channel = DMA_CHANNEL_4;
	DMA2_HandleStructure_UART1_RX.Init.Direction = DMA_PERIPH_TO_MEMORY;
	DMA2_HandleStructure_UART1_RX.Init.PeriphInc = DMA_PINC_DISABLE;
	DMA2_HandleStructure_UART1_RX.Init.MemInc = DMA_MINC_ENABLE;
	DMA2_HandleStructure_UART1_RX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	DMA2_HandleStructure_UART1_RX.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	DMA2_HandleStructure_UART1_RX.Init.Mode = DMA_NORMAL;
	DMA2_HandleStructure_UART1_RX.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	DMA2_HandleStructure_UART1_RX.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init (&DMA2_HandleStructure_UART1_RX);
	__HAL_LINKDMA(&UART1_HandleStructure, hdmarx, DMA2_HandleStructure_UART1_RX);
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 3);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	/* Peripheral DMA init*/										//UART2 TX
	DMA1_HandleStructure_UART2_TX.Instance = DMA1_Stream6;
	DMA1_HandleStructure_UART2_TX.Init.Channel = DMA_CHANNEL_4;
	DMA1_HandleStructure_UART2_TX.Init.Direction = DMA_MEMORY_TO_PERIPH;
	DMA1_HandleStructure_UART2_TX.Init.PeriphInc = DMA_PINC_DISABLE;
	DMA1_HandleStructure_UART2_TX.Init.MemInc = DMA_MINC_ENABLE;
	DMA1_HandleStructure_UART2_TX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	DMA1_HandleStructure_UART2_TX.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	DMA1_HandleStructure_UART2_TX.Init.Mode = DMA_NORMAL;
	DMA1_HandleStructure_UART2_TX.Init.Priority = DMA_PRIORITY_HIGH;
	DMA1_HandleStructure_UART2_TX.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init (&DMA1_HandleStructure_UART2_TX);
	__HAL_LINKDMA(&UART2_HandleStructure, hdmatx, DMA1_HandleStructure_UART2_TX);

	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

	/* Peripheral DMA init*/										//UART2 RX
	DMA1_HandleStructure_UART2_RX.Instance = DMA1_Stream5;
	DMA1_HandleStructure_UART2_RX.Init.Channel = DMA_CHANNEL_4;
	DMA1_HandleStructure_UART2_RX.Init.Direction = DMA_PERIPH_TO_MEMORY;
	DMA1_HandleStructure_UART2_RX.Init.PeriphInc = DMA_PINC_DISABLE;
	DMA1_HandleStructure_UART2_RX.Init.MemInc = DMA_MINC_ENABLE;
	DMA1_HandleStructure_UART2_RX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	DMA1_HandleStructure_UART2_RX.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	DMA1_HandleStructure_UART2_RX.Init.Mode = DMA_CIRCULAR;	//change from circular to normal 28 May 2019
	DMA1_HandleStructure_UART2_RX.Init.Priority = DMA_PRIORITY_HIGH;
	DMA1_HandleStructure_UART2_RX.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init (&DMA1_HandleStructure_UART2_RX);
	__HAL_LINKDMA(&UART2_HandleStructure, hdmarx, DMA1_HandleStructure_UART2_RX);
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

	/* Peripheral DMA init*/										//UART3 TX
	DMA1_HandleStructure_UART3_TX.Instance = DMA1_Stream3;
	DMA1_HandleStructure_UART3_TX.Init.Channel = DMA_CHANNEL_4;
	DMA1_HandleStructure_UART3_TX.Init.Direction = DMA_MEMORY_TO_PERIPH;
	DMA1_HandleStructure_UART3_TX.Init.PeriphInc = DMA_PINC_DISABLE;
	DMA1_HandleStructure_UART3_TX.Init.MemInc = DMA_MINC_ENABLE;
	DMA1_HandleStructure_UART3_TX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	DMA1_HandleStructure_UART3_TX.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	DMA1_HandleStructure_UART3_TX.Init.Mode = DMA_NORMAL;
	DMA1_HandleStructure_UART3_TX.Init.Priority = DMA_PRIORITY_LOW;
	DMA1_HandleStructure_UART3_TX.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init (&DMA1_HandleStructure_UART3_TX);
	__HAL_LINKDMA(&UART3_HandleStructure, hdmatx, DMA1_HandleStructure_UART3_TX);
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 5);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

	/* Peripheral DMA init*/										//UART3 RX
	DMA1_HandleStructure_UART3_RX.Instance = DMA1_Stream1;
	DMA1_HandleStructure_UART3_RX.Init.Channel = DMA_CHANNEL_4;
	DMA1_HandleStructure_UART3_RX.Init.Direction = DMA_PERIPH_TO_MEMORY;
	DMA1_HandleStructure_UART3_RX.Init.PeriphInc = DMA_PINC_DISABLE;
	DMA1_HandleStructure_UART3_RX.Init.MemInc = DMA_MINC_ENABLE;
	DMA1_HandleStructure_UART3_RX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	DMA1_HandleStructure_UART3_RX.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	DMA1_HandleStructure_UART3_RX.Init.Mode = DMA_CIRCULAR;	//change from circular 28 May 2019
	DMA1_HandleStructure_UART3_RX.Init.Priority = DMA_PRIORITY_HIGH;
	DMA1_HandleStructure_UART3_RX.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init (&DMA1_HandleStructure_UART3_RX);
	__HAL_LINKDMA(&UART3_HandleStructure, hdmarx, DMA1_HandleStructure_UART3_RX);
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 6);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

	DMA2_HandleStructure_UART1_TX.Instance->CR &= ~DMA_SxCR_HTIE;//disable half TX cplt interrupt
	DMA1_HandleStructure_UART2_TX.Instance->CR &= ~DMA_SxCR_HTIE;//disable half TX cplt interrupt
	DMA1_HandleStructure_UART3_TX.Instance->CR &= ~DMA_SxCR_HTIE;//disable half TX cplt interrupt

	__HAL_UART_ENABLE_IT(&UART1_HandleStructure, UART_IT_TXE);
}

UART_HandleTypeDef * CommsManager::getMotorUART() {
	return &UART1_HandleStructure;
}

UART_HandleTypeDef * CommsManager::getRemoteUART() {
	return &UART2_HandleStructure;
}

UART_HandleTypeDef * CommsManager::getAccUART() {
	return &UART3_HandleStructure;
}

DMA_HandleTypeDef * CommsManager::getMotorDMA(COMM_DIR tx_rx) {
	if (tx_rx == TX) {
		return &DMA2_HandleStructure_UART1_TX;
	} else if (tx_rx == RX) {
		return &DMA2_HandleStructure_UART1_RX;
	} else {
		return 0;
	}
}

DMA_HandleTypeDef * CommsManager::getRemoteDMA(COMM_DIR tx_rx) {
	if (tx_rx == TX) {
		return &DMA1_HandleStructure_UART2_TX;
	} else if (tx_rx == RX) {
		return &DMA1_HandleStructure_UART2_RX;
	} else {
		return 0;
	}
}

DMA_HandleTypeDef * CommsManager::getAccDMA(COMM_DIR tx_rx) {
	if (tx_rx == TX) {
		return &DMA1_HandleStructure_UART3_TX;
	} else if (tx_rx == RX) {
		return &DMA1_HandleStructure_UART3_RX;
	} else {
		return 0;
	}
}

HAL_StatusTypeDef CommsManager::transmitMotors(uint8_t * data, uint8_t size, uint16_t timeout, enum UART_TRANSMISSION_TYPE type) {
	HAL_StatusTypeDef status;
	if (UART1_HandleStructure.gState == HAL_UART_STATE_READY) {
			if(type == DMA){
				status = HAL_UART_Transmit_DMA(&UART1_HandleStructure, data, size);
			}else if(type == INTERRUPT){
				status = HAL_UART_Transmit_IT(&UART1_HandleStructure, data, size);
			}else if(type == BLOCKING){
				status = HAL_UART_Transmit(&UART1_HandleStructure, data, size, timeout);
			}else{
				status = HAL_ERROR;
			}
		} else {
			status = HAL_ERROR;
		}

	return status;
}

HAL_StatusTypeDef CommsManager::transmitRemote(uint8_t * data, uint8_t size, uint16_t timeout, enum UART_TRANSMISSION_TYPE type) {
		HAL_StatusTypeDef status;
		if(type == DMA){
			status = HAL_UART_Transmit_DMA(&UART2_HandleStructure, data, size);
		}else if(type == INTERRUPT){
			status = HAL_UART_Transmit_IT(&UART2_HandleStructure, data, size);
		}else if(type == BLOCKING){
			status = HAL_UART_Transmit(&UART2_HandleStructure, data, size, timeout);
		}else{
			status = HAL_ERROR;
		}
		return status;
}

HAL_StatusTypeDef CommsManager::transmitAcc(uint8_t * data, uint8_t size, uint16_t timeout, enum UART_TRANSMISSION_TYPE type) {
	if (UART3_HandleStructure.gState == HAL_UART_STATE_READY) {
			if(type == DMA){
				return HAL_UART_Transmit_DMA(&UART3_HandleStructure, data, size);
			}else if(type == INTERRUPT){
				return HAL_UART_Transmit_IT(&UART3_HandleStructure, data, size);
			}else if(type == BLOCKING){
				return HAL_UART_Transmit(&UART3_HandleStructure, data, size, timeout);
			}else{
				return HAL_ERROR;
			}
		} else {
			return HAL_ERROR;
		}
}

HAL_StatusTypeDef CommsManager::receiveMotors(uint8_t * data, uint8_t size, uint16_t timeout, enum UART_TRANSMISSION_TYPE type) {
	if (UART1_HandleStructure.RxState == HAL_UART_STATE_READY) {
			if(type == DMA){
				return HAL_UART_Receive_DMA(&UART1_HandleStructure, data, size);
			}else if(type == INTERRUPT){
				return HAL_UART_Receive_IT(&UART1_HandleStructure, data, size);
			}else if(type == BLOCKING){
				return HAL_UART_Receive(&UART1_HandleStructure, data, size, timeout);
			}else{
				return HAL_ERROR;
			}
		} else {
			return HAL_ERROR;
		}
}

HAL_StatusTypeDef CommsManager::receiveRemote(uint8_t * data, uint8_t size, uint16_t timeout, enum UART_TRANSMISSION_TYPE type) {
	if (UART2_HandleStructure.RxState == HAL_UART_STATE_READY) {
		if(type == DMA){
			return HAL_UART_Receive_DMA(&UART2_HandleStructure, data, size);
		}else if(type == INTERRUPT){
			return HAL_UART_Receive_IT(&UART2_HandleStructure, data, size);
		}else if(type == BLOCKING){
			return HAL_UART_Receive(&UART2_HandleStructure, data, size, timeout);
		}else{
			return HAL_ERROR;
		}
	} else {
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef CommsManager::receiveAcc(uint8_t * data, uint8_t size, uint16_t timeout, enum UART_TRANSMISSION_TYPE type) {
	if (UART3_HandleStructure.RxState == HAL_UART_STATE_READY) {
			if(type == DMA){
				return HAL_UART_Receive_DMA(&UART3_HandleStructure, data, size);
			}else if(type == INTERRUPT){
				return HAL_UART_Receive_IT(&UART3_HandleStructure, data, size);
			}else if(type == BLOCKING){
				return HAL_UART_Receive(&UART3_HandleStructure, data, size, timeout);
			}else{
				return HAL_ERROR;
			}
		} else {
			return HAL_ERROR;
		}
}

