/**
 * @file Remote.cpp
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief Remote class functions.
 **/

#include "Remote.h"

Remote::Remote(CommsManager * comms){
	this->comms = comms;
}


Remote::~Remote(void){

}

void Remote::sendASCII(char * msg){
	int count = 0;

	while(msg[count] != '\0'){
		count++;
	}

	uint8_t data[count+3];
	data[0] = 255;
	data[1] = 252;
	data[2] = count;

	for(int i = 3; i < count+3; i++){
		data[i] = msg[i-3];
	}

	comms->transmitRemote(data, count+3, 20*count, DMA);
}

bool Remote::sendAcc(double pitch, double roll, double yaw, uint16_t timestamp, double slopePitch, double slopeRoll){

	this->sending = true;

	dma_acc_transmit[0] = 255;
	dma_acc_transmit[1] = 254;

	floatToBytes(timestamp, &dma_acc_transmit[2]);

	floatToBytes(pitch, &dma_acc_transmit[6]);

	floatToBytes(roll, &dma_acc_transmit[10]);

	floatToBytes(yaw, &dma_acc_transmit[14]);

	floatToBytes(slopePitch, &dma_acc_transmit[18]);

	floatToBytes(slopeRoll, &dma_acc_transmit[22]);

		if(__HAL_UART_GET_FLAG(comms->getRemoteUART(),UART_FLAG_TXE)){
			if(comms->transmitRemote(dma_acc_transmit, 26, 0, DMA) == HAL_OK){
				return true;
			}else{
				return false;
			}
		}else{
			return false;
		}
}

void Remote::sendCharging(bool charging){
	uint8_t data[] = {255, 249, 249, (uint8_t)charging};
	comms->transmitRemote(data, 4, 1000, DMA);
}

void Remote::sendError(uint8_t errorLoc, uint8_t errorCode){
	dma_error_transmit[0] = 255;
	dma_error_transmit[1] = 253;
	dma_error_transmit[2] =	errorLoc;
	dma_error_transmit[3] = errorCode;

	comms->transmitRemote(dma_error_transmit, 4, 1000, DMA);
}

void Remote::receiveRemoteData(uint8_t size){
	comms->receiveRemote(dma_dataPacket, size, 0, DMA);
}

void Remote::processRemoteData(){
	if( isChecksumValid() ){ //check if received data is valid based on checksum value

			this->newData = true;

			uint8_t returnData[2] = {255, 0};
			for(int i = 0; i < 9; i++){
				returnData[1] += dma_dataPacket[i];
			}

			comms->transmitRemote(returnData, 2, 10, DMA);

			if(dma_dataPacket[0] == 0x05){			//pid controller updates


			}else if(dma_dataPacket[0] == 0x01){	//direction command

				switch(dma_dataPacket[1]){
				case 0:
					this->direction = -10.0;
					break;

				case 1:
					this->direction = 0;
					break;

				case 2:
					this->direction = 3.14159;
					break;

				case 3:
					this->direction = -1.5708;
					break;

				case 4:
					this->direction = 1.5708;
					break;

				case 5:
					this->direction = 0.785398;
					break;

				case 6:
					this->direction = 2.35619;
					break;

				case 7:
					this->direction = -0.785398;
					break;

				case 8:
					this->direction = -2.35619;
					break;
				}

				this->twistBody = ((dma_dataPacket[2]/100.0)-1)*1.0;

				if(dma_dataPacket[3] == 0x02){	//speed command
					this->speed = (dma_dataPacket[4]/100.0);
					if(this->speed == 0){
						this->speed = 1;
					}
				}

				if(dma_dataPacket[5] == 0x03){	//bodyheight command
					this->bodyHeight = 280*dma_dataPacket[6]/100.0;
				}

				if(dma_dataPacket[7] == 0x04){	//function command
					this->function = dma_dataPacket[8];
				}
			}
		}else{
			trace_printf("REMOTE CHECKSUM ERROR\n");
		}
}

bool Remote::isChecksumValid(){
	long checksumTemp = 0;
	for(int i = 0; i < 9; i++){
		checksumTemp += this->dma_dataPacket[i];
	}

	uint8_t check = (uint8_t)checksumTemp;

	if(check == (uint8_t)this->dma_dataPacket[9]){
		return true;
	}else{
		return false;
	}
}

void Remote::floatToBytes(float value, uint8_t * bytes){
	*(float*)(bytes) = value;	// convert float into bytes
}

float Remote::bytesToFloat(uint8_t * bytes){
	float temp = *(float*)(bytes);
	return temp;
}

bool Remote::successfulPing(){
	return pingSuccess;
}

