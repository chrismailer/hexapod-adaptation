/**
 * @file Motor.cpp
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief Motor class functions.
 **/


#include "Motor.h"

Motor::Motor(uint8_t IDVal, CommsManager * comms) : ID(IDVal){
	this->comms = comms;
}

Motor::~Motor(void){

}


void Motor::setGoalPositionSpeed(double theta, double speed){
	this->calcPositionBits(theta);
	this->calcSpeedBits(speed);
}

void Motor::calcPositionBits(double theta){		//now in radians
	double pos = (theta+2.61799)*195.569883766 ; //was 150 deg and *3.41

	if(this->ID%3==0){
		pos = (theta-2.61799)*-195.569883766;
	}

	if(pos < 0){
		pos = 0;
	}else if(pos > 1023){
		pos = 1023;
	}

	this->goalMovementBits[0] = (uint16_t)pos & 0xff;
	this->goalMovementBits[1] = (uint16_t)pos >> 8;
}

void Motor::calcSpeedBits(double theta_dot){	//rpm
	double speed = theta_dot*1024.0/114.0;
	this->goalMovementBits[2] = (uint16_t)(speed < 1 ? 1 : speed) & 0xff;
	this->goalMovementBits[3] = (uint16_t)(speed < 1 ? 1 : speed) >> 8;
}

void Motor::setPositionBits(uint16_t pos){
	this->goalMovementBits[0] = (uint16_t)pos & 0xff;
	this->goalMovementBits[1] = (uint16_t)pos >> 8;
}

void Motor::setSpeedBits(uint16_t speed){
	this->goalMovementBits[2] = (uint16_t)speed & 0xff;
	this->goalMovementBits[3] = (uint16_t)speed >> 8;
}

uint8_t * Motor::getGoalMovementBits(void){
	return this->goalMovementBits;
}

void Motor::move(void){
	this->writeReg(GOAL_POSITION_L, this->goalMovementBits, 4);
}

void Motor::writeReg(enum CONTROL_TABLE reg, FunctionalState state){
	uint8_t data[1] = {state};
	this->writeReg(reg, data, 1);
}

void Motor::writeReg(enum CONTROL_TABLE reg, uint8_t * data, uint8_t dataLength){
	//write data only
	uint8_t length = dataLength + 3;

	uint8_t packet[length+4];

	packet[0] = packet[1] = 0xFF;
	packet[2] = this->ID;
	packet[3] = length;
	packet[4] = WRITE_DATA;
	packet[5] = reg;

	for(int j = 0; j < dataLength; j++){
		packet[6+j] = data[j];
	}

	packet[length+3] = 0;
	for(int i = 2; i < length+3; i++){
		packet[length+3] += packet[i];
	}

	packet[length+3] = ~packet[length+3];

	//HAL_Delay(2);			//without this DMA transmission and reception of data doesn't work...

	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);

	comms->transmitMotors(packet, length+4, 100, BLOCKING);
}

bool Motor::readReg(enum CONTROL_TABLE reg, uint8_t dataLength){
	//this->reading = true;
	if(this->readPacket[2] != this->ID){
		this->readDataSize = dataLength;
		//read data only
		uint8_t length = 4;

		readPacket[0] = readPacket[1] = 0xFF;
		readPacket[2] = this->ID;
		readPacket[3] = length;
		readPacket[4] = READ_DATA;
		readPacket[5] = reg;

		readPacket[6] = dataLength;

		readPacket[length+3] = 0;
		for(int i = 2; i < length+3; i++){
			readPacket[length+3] += readPacket[i];
		}

		readPacket[length+3] = ~readPacket[length+3];
	}

	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
	for(int i = 0; i < 100000; i++);
	if(comms->transmitMotors(readPacket, 4+4, 0, DMA) == HAL_OK){
		if(comms->receiveMotors(this->controlTable,this->readDataSize+6,0,DMA) == HAL_OK){
			return true;
		}else{
			return false;
		}
	}else{
		HAL_UART_DMAStop(comms->getMotorUART());
		return false;
	}

	//for(int i = 0; i < 100000; i++);

}

bool Motor::readTable(){
	//this->reading = true;
	if(this->readPacket[2] != this->ID){
		uint8_t dataLength = 50;
		enum CONTROL_TABLE reg = MODEL_NUMBER_L;

		this->readDataSize = dataLength;
		//read data only
		uint8_t length = 4;

		readPacket[0] = readPacket[1] = 0xFF;
		readPacket[2] = this->ID;
		readPacket[3] = length;
		readPacket[4] = READ_DATA;
		readPacket[5] = reg;

		readPacket[6] = dataLength;

		readPacket[length+3] = 0;
		for(int i = 2; i < length+3; i++){
			readPacket[length+3] += readPacket[i];
		}

		readPacket[length+3] = ~readPacket[length+3];
	}

	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
	if(comms->transmitMotors(readPacket, 4+4, 100, BLOCKING) == HAL_OK){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
		if(comms->receiveMotors(this->controlTable,this->readDataSize+6,1000, BLOCKING) == HAL_OK){
			return true;
		}else{
			return false;
		}
	}else{
		return false;
	}
}

void Motor::clearControlTable(){
	for(int i = 0; i < 56; i++){
		this->controlTable[i] = 0;
	}
}

void Motor::resetErrors(){
	uint8_t data[] = {255,3};
	this->writeReg(TORQUE_LIMIT_L, data, 2);

	uint8_t data2[] = {0,0};
	this->writeReg(TORQUE_ENABLE, data2, 2);

	for(int i = 0; i < 100000;i++);	//delay (HAL_Delay causes timer issues)

	uint8_t data3[] = {1,0};
	this->writeReg(TORQUE_ENABLE, data3, 2);

	for(int i = 0; i < 100000;i++);	//delay (HAL_Delay causes timer issues)
}

bool Motor::pingBool(){
	uint8_t packet[2+4];

	packet[0] = packet[1] = 0xFF;
	packet[2] = this->ID;
	packet[3] = 2;
	packet[4] = PING;


	packet[2+3] = 0;
	for(int i = 2; i < 2+3; i++){
		packet[2+3] += packet[i];
	}

	packet[2+3] = ~packet[2+3];

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	comms->transmitMotors(packet,sizeof(packet)/sizeof(packet[0]), 100, BLOCKING);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	comms->receiveMotors(this->pingStatusPacket, 6, 100, BLOCKING);

	return (this->pingStatusPacket[2] == this->ID);

}

uint8_t * Motor::pingData(){
	uint8_t packet[2+4];

	packet[0] = packet[1] = 0xFF;
	packet[2] = this->ID;
	packet[3] = 2;
	packet[4] = PING;


	packet[2+3] = 0;
	for(int i = 2; i < 2+3; i++){
		packet[2+3] += packet[i];
	}

	packet[2+3] = ~packet[2+3];

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	comms->transmitMotors(packet, sizeof(packet)/sizeof(packet[0]), 100, BLOCKING);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	if(comms->receiveMotors(this->pingStatusPacket, 6, 100, BLOCKING)==HAL_TIMEOUT){
		this->pingStatusPacket[2] = 0;
	}

	return this->pingStatusPacket;

}
