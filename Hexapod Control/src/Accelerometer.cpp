/**
 * @file Accelerometer.cpp
 * @author Ross Christopher (CHRROS005)
 * @date 2019
 * @brief Accelerometer class functions.
 **/

#include "Accelerometer.h"

// Private Functions

float Accelerometer::bytesToFloat(int startIndex){
	uint8_t bytes[4] = {this->dma_dataPacket[startIndex+3], this->dma_dataPacket[startIndex+2], this->dma_dataPacket[startIndex+1], this->dma_dataPacket[startIndex]};
	return *(float*)(bytes);
}

bool Accelerometer::isChecksumValid(){
	long temp = 0;
	for(int i = 1; i < 55; i++){
		temp += this->dma_dataPacket[i];
	}
	if( ((uint8_t)(temp & 0xFF) == 0)  && (temp != 0)){
		return true;
	}else{
		return false;
	}
}

// Public Functions

Accelerometer::Accelerometer(CommsManager * comms){
	this->comms = comms;
}


Accelerometer::~Accelerometer(void){

}

void Accelerometer::requestAccelData(){
	if(this->comms->getAccUART()->gState == HAL_UART_STATE_READY){
		this->accel_transmission = DATA;
		this->comms->transmitAcc(reqData, 5, 0, DMA);
	}
}

void Accelerometer::receiveAccelData(){
		if(this->accel_transmission == CONFIG){
			this->comms->receiveAcc(dma_infoPacket, 5, 100, BLOCKING);
			this->accel_transmission = DATA;
		}else if(this->accel_transmission == DATA){
			this->comms->receiveAcc(dma_dataPacket, 55, 0, DMA);
		}
}

void Accelerometer::processAccelData(){
	if( this->isChecksumValid()){

		//uint8_t accXbytes[4] = {this->dma_dataPacket[7], this->dma_dataPacket[6], this->dma_dataPacket[5], this->dma_dataPacket[4]};
		//this->accX = (*(float*)(accXbytes) - 90.0)*M_PI/180.0;

		//uint8_t accYbytes[4] = {this->dma_dataPacket[11], this->dma_dataPacket[10], this->dma_dataPacket[9], this->dma_dataPacket[8]};
		//this->accY = (*(float*)(accYbytes) - 90.0)*M_PI/180.0;

		//uint8_t accZbytes[4] = {this->dma_dataPacket[15], this->dma_dataPacket[14], this->dma_dataPacket[13], this->dma_dataPacket[12]};
		//this->accZ = (*(float*)(accZbytes) - 90.0)*M_PI/180.0;

		//gyro data from [16] - [27]

		//mag data from [28] - [39]

		double tempRoll = (this->bytesToFloat(40) - 90.0)*M_PI/180.0;

		double tempPitch = (this->bytesToFloat(44))*M_PI/180.0;

		this->roll = tempPitch;

		this->pitch = tempRoll;

		this->yaw  = (this->bytesToFloat(48))*M_PI/180.0;

		this->timestamp = (this->dma_dataPacket[52] << 8 | this->dma_dataPacket[53]);

		// rotate roll and pitch to align with hexapod (or swap pitch and roll???)
		/*double ANGLE = 90.0*(M_PI/60.0);
		this->roll  = tempRoll*cos(ANGLE) - tempPitch*sin(ANGLE);
		this->pitch = tempRoll*sin(ANGLE) + tempPitch*sin(ANGLE);	// this->roll must be original roll angle not updated angle.*/

	}else if(this->accel_transmission == CONFIG){
		this->accel_transmission = DATA;
	}else{
		trace_printf("ACC CHECKSUM ERROR\n");
	}
}

void Accelerometer::setMeasurementMode(){
	this->accel_transmission = CONFIG;
	if(this->comms->getAccUART()->gState == HAL_UART_STATE_READY){
		this->comms->transmitAcc(gotoMeasurement, 5, 1000, BLOCKING);
		this->receiveAccelData();
	}
}


void Accelerometer::setConfigMode(){
	if(this->comms->getAccUART()->gState == HAL_UART_STATE_READY){
		accel_transmission = CONFIG;
		this->comms->transmitAcc(gotoConfig, 5, 1000, BLOCKING);
		this->receiveAccelData();
	}
}

void Accelerometer::resetHeading(bool confirm){
	if(confirm && this->comms->getAccUART()->gState == HAL_UART_STATE_READY){
		this->accel_transmission = CONFIG;
		this->comms->transmitAcc(resetHeadingData, 7, 1000, BLOCKING);
		this->receiveAccelData();
		this->accel_transmission = DATA;
		this->receiveAccelData();
	}
}
