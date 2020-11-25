/**
 * @file main.cpp
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief main function
 **/

#include "CommsManager.h"
#include "Hexapod.h"

/** @brief Main Hexapod object.  **/
Hexapod HEX;

void main(){

	HEX.Initialise();
	HEX.startTimers();

	for(int i = 0; i < 10000000;i++);	//delay (HAL_Delay causes timer issues)
	HEX.stand();
	for(int i = 0; i < 10000000;i++);	//delay (HAL_Delay causes timer issues)

	//HAL_Delay(5000);

	/*uint8_t data[] = {250};
	HEX.writeRegAll(RETURN_DELAY_TIME, data, 1);*/

	/*uint8_t data2[] = {32, 32};
	HEX.writeRegAll(CW_COMPLIANCE_SLOPE, data2, 2);*/

	HEX.pidPitch->togglePID(false);
	HEX.pidRoll->togglePID(false);
	HEX.pidYaw->togglePID(false);

	//HEX.pidPitch->setParameters(1,0,0);

	//uint8_t data3[] = {180,0,75,3};		//correct for motor 1 not hitting body.		// ALL IGNORING OTHER LEG POSITIONS
	//uint8_t data3[] = {75,0,245,3};		//correct for motor 2 not hitting body.
	//uint8_t data3[] = {0,0,190,3};		//correct for motor 3 not hitting body.
	/*for(int i = 0; i < 6; i++){			//change all legs
		HEX.legs[i]->motors[2]->writeReg(CW_ANGLE_LIMIT_L, data3, 4);
	}*/

	//HEX.liftOntoBase();
	HEX.timersReady = true;
	HEX.makePath();

	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	//Move to starting gait position
	for(int i = 0; i < 10000000;i++);
	HEX.setNextPathPoint();
	HEX.setSpeedRPM(20,20,20);
	HEX.move(DMA);
	for(int i = 0; i < 100000000;i++);

	while(true){	//super loop
		if(HEX.newPosition && !HEX.charging && !HEX.lowPowerMode && (HEX.elapsedTime < 5.0)){
			HEX.newPosition = false;
			if(HEX.direction != -10){
				HEX.setNextPathPoint();
			}
			HEX.move(DMA);
		}
	}// end super loop
}//end main

#ifdef __cplusplus
extern "C"
{
#endif		//EXTERN "C"

/**
 * @brief Timer 2 interrupt for path position control.
 *
 * @details
 **/
void TIM2_IRQHandler(void){
	HAL_TIM_IRQHandler(HEX.getTIM(TIM2));
	if(HEX.timersReady && !HEX.charging){
		if(HEX.direction != -10){
			HEX.newPosition = true;
		}
	}
}

/**
 * @brief Timer 3 interrupt for IMU and ADC reading.
 *
 * @details
 **/
void TIM3_IRQHandler(void) {				// Accelerometer Reading, ADC Reading
	HAL_TIM_IRQHandler(HEX.getTIM(TIM3));
	if(HEX.timersReady){

		if((HEX.TIM3Counter % (10*(!HEX.lowPowerMode?10:20))) == 0 && HEX.TIM3Counter != 0){		//every 10 seconds if not in low power mode, 60 seconds otherwise
			struct CHARGER charger = HEX.readCharger();
			switch(charger.status){
				case FAULT:
					HEX.charging = false;
					HEX.enterLowPowerMode();
					HEX.remote->sendASCII("*****CHARGING FAULT*****");
					while(1);		//hang
					break;
				case NOT_CHARGING:
					HEX.charging = false;
					if(HEX.shouldCharging){
						HEX.startCharging();
					}
					break;
				case CHARGING_CC:
					HEX.disableMotors();
					HEX.charging = true;
					break;
				case CHARGING_CV:
					HEX.disableMotors();
					HEX.charging = true;
					break;
				default:
					break;
			}

			if(charger.voltage <= 20.0 && charger.voltage > 10.0){
				HEX.remote->sendASCII("LOW BATTERY");
				HEX.lowBatteryCount++;

				if(HEX.lowBatteryCount >= 6){	//make sure that battery is actually low and not just a spike by checking for 1 minute.
					if(!HEX.lowPowerMode){
						HEX.enterLowPowerMode();
					}
					if(NOT_CHARGING){
						HEX.shouldCharging = true;
						HEX.startCharging();
					}
				}
			}else{
				HEX.lowBatteryCount = 0;
				if(HEX.lowPowerMode){
					HEX.exitLowPowerMode();
				}
			}

			uint8_t data[10] = {255, 249, charger.status, 0,0,0,0, 0,0,0};
			HEX.remote->floatToBytes(charger.voltage, &data[3]);
			long checksumTemp = 0;
			for(int i = 0; i < 9; i++){
				checksumTemp += data[i];
			}
			data[9] = (uint8_t)checksumTemp;

			HEX.remote->sending = false;
			if(!HEX.remote->sending){
				HEX.remote->sending = true;
				HEX.comms.transmitRemote(data, 10, 100, DMA);
			}

			//HEX.TIM3Counter = 0;
		}

		/*if((HEX.TIM3Counter % (200*10)) == 0){		//every 10 seconds
			//HEX.readInternalTemp();
			if((HEX.currentTemperature - HEX.startingTemperature)*(HEX.currentTemperature - HEX.startingTemperature) > 40.0*40.0 || HEX.currentTemperature > 120.0){
				//HEX.fullSystemReset(true);	//over-heating (also add battery shutdown/motor shutdown)
			}
		}*/

		// PING COMPUTER
		if((HEX.TIM3Counter % (10*5)) == 0 && HEX.TIM3Counter != 0 && !HEX.lowPowerMode){	//every 5 seconds, brackets around (10*time) are important!
			if (HEX.remote->pingSuccess) {
				HEX.remote->pingSuccess = false;
				uint8_t data[2] = {255, 250};
				HEX.remote->sending = true;
				HEX.comms.transmitRemote(data, 2, 100, DMA);
				HEX.comms.remoteFailureCount = 0;
			} else {
				//trace_printf("COMMS FAILURE\n");
				//HEX.remote->sendASCII("COMMS FAILURE");
				if (HEX.comms.remoteFailureCount >= 10) {
					HEX.stop();
					HEX.comms.remoteFailureCount = 0;
					//HEX.remote->sendASCII("RESETTING...");
					//trace_printf("RESETTING...\n");
					//HEX.fullSystemReset(true);
				} else {
					HEX.comms.remoteFailureCount++;
				}
			}
		}


		if((HEX.TIM3Counter % (uint8_t)(10*HEX.IMUTransFreq)) == 0 && HEX.TIM3Counter != 0 && !HEX.charging && !HEX.lowPowerMode){	//every 1 seconds, brackets around (10*time) are important!
			HEX.remote->sending = false;
			if(!HEX.remote->sending){
				HEX.remote->sending = true;
				HEX.remote->sendAcc(HEX.accel->pitch, HEX.accel->roll, HEX.accel->yaw, HEX.accel->timestamp, HEX.slopePitch, HEX.slopeRoll);
			}
		}

		if(!HEX.shouldCharging && !HEX.lowPowerMode){
			HEX.accel->requestAccelData();
		}


		HEX.TIM3Counter++;

	}
}

/**
 * @brief UART transmit complete interrupt.
 *
 * @details
 *
 * @param UART_HandleStructure Pointer to the UART struct which caused the error interrupt.
 **/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef * UART_HandleStructure){
	__HAL_UART_CLEAR_FLAG(UART_HandleStructure, UART_FLAG_TC);
	//__HAL_UART_CLEAR_OREFLAG(UART_HandleStructure);

	UART_HandleStructure->gState = HAL_UART_STATE_READY;

	if(UART_HandleStructure->Instance == UART3){
		//accelerometer transmit complete
	}else if(UART_HandleStructure->Instance == UART2){
		//remote transmit complete
		if(UART_HandleStructure->pTxBuffPtr[0]==255 && UART_HandleStructure->pTxBuffPtr[1]==248 && (UART_HandleStructure->pTxBuffPtr[9]==252 || UART_HandleStructure->pTxBuffPtr[9]==253 ) ){
			//HEX.remote->sendCharging(HEX.shouldCharging);
		}
	}else if(UART_HandleStructure->Instance == UART1){
		//HEX.UART1_Toggle_Read();
	}
}

/**
 * @brief UART receive complete interrupt.
 *
 * @details
 *
 * @param UART_HandleStructure Pointer to the UART struct which caused the error interrupt.
 **/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * UART_HandleStructure){
	__HAL_UART_CLEAR_FLAG(UART_HandleStructure, UART_FLAG_RXNE);
	//__HAL_UART_CLEAR_OREFLAG(UART_HandleStructure);

	UART_HandleStructure->RxState = HAL_UART_STATE_READY;
	UART_HandleStructure->gState = HAL_UART_STATE_READY;

	if(UART_HandleStructure->Instance == UART1){

	}else if(UART_HandleStructure->Instance == UART2){						//remote data
		if(UART_HandleStructure->pRxBuffPtr[0] == 0x01 && UART_HandleStructure->pRxBuffPtr[7] == 0x04){	//remote movement data
			HEX.remote->processRemoteData();
			HEX.update();
		}else if(UART_HandleStructure->pRxBuffPtr[0] == 55){	//remote ping data
			HEX.remote->pingSuccess = true;
		}else if(UART_HandleStructure->pRxBuffPtr[0] == 255 && UART_HandleStructure->pRxBuffPtr[1] == 248){	//BASE TO HEXAPOD
			if(UART_HandleStructure->pRxBuffPtr[2] == 55){	//command incoming
				if(UART_HandleStructure->pRxBuffPtr[3] == 1){	//start charging
					HEX.shouldCharging = true;
					HEX.startCharging();
				}else if(UART_HandleStructure->pRxBuffPtr[3] == 0){	//stop charging
					HEX.shouldCharging = false;
					HEX.stopCharging();
				}
			}else{
				// status and voltage incoming
			}
		}else if(UART_HandleStructure->pRxBuffPtr[0] == 255 && UART_HandleStructure->pRxBuffPtr[1] == 250){// toggle motors command
			bool toggleMotors = (bool)UART_HandleStructure->pRxBuffPtr[2];
			if (toggleMotors) {
				HEX.enableMotors();
			} else {
				HEX.disableMotors();
			}
		}else if(UART_HandleStructure->pRxBuffPtr[0] == 255 && UART_HandleStructure->pRxBuffPtr[1] == 251){	//disable/enable pitch/roll PID
			bool togglePID = (bool)UART_HandleStructure->pRxBuffPtr[2];
			uint8_t pitchRoll = UART_HandleStructure->pRxBuffPtr[3];

			if(!HEX.liftedForBase){
				if(pitchRoll == 1){	//pitch
					HEX.pidPitch->togglePID(togglePID);
				}else if(pitchRoll == 2){	// roll
					HEX.pidRoll->togglePID(togglePID);
				}else if(pitchRoll == 3){
					HEX.pidPitch->togglePID(togglePID);
					HEX.pidRoll->togglePID(togglePID);
				}
			}
		}else if(UART_HandleStructure->pRxBuffPtr[0] == 255 && UART_HandleStructure->pRxBuffPtr[1] == 222){ // set PID parameters command
				if(UART_HandleStructure->pRxBuffPtr[2] == 1){//pitch
					if(UART_HandleStructure->pRxBuffPtr[3]==1){//kp
						HEX.pidPitch->setKp(HEX.remote->bytesToFloat(&UART_HandleStructure->pRxBuffPtr[4]));
					}else if(UART_HandleStructure->pRxBuffPtr[3]==2){//ki
						HEX.pidPitch->setKi(HEX.remote->bytesToFloat(&UART_HandleStructure->pRxBuffPtr[4]));
					}else if(UART_HandleStructure->pRxBuffPtr[3]==3){//kd
						HEX.pidPitch->setKd(HEX.remote->bytesToFloat(&UART_HandleStructure->pRxBuffPtr[4]));
					}
				}else if(UART_HandleStructure->pRxBuffPtr[2] == 2){//roll
					if(UART_HandleStructure->pRxBuffPtr[3]==1){//kp
						HEX.pidRoll->setKp(HEX.remote->bytesToFloat(&UART_HandleStructure->pRxBuffPtr[4]));
					}else if(UART_HandleStructure->pRxBuffPtr[3]==2){//ki
						HEX.pidRoll->setKi(HEX.remote->bytesToFloat(&UART_HandleStructure->pRxBuffPtr[4]));
					}else if(UART_HandleStructure->pRxBuffPtr[3]==3){//kd
						HEX.pidRoll->setKd(HEX.remote->bytesToFloat(&UART_HandleStructure->pRxBuffPtr[4]));
						for(int i = 0; i < 1; i++);
					}
				}else if(UART_HandleStructure->pRxBuffPtr[2] == 3){//both
					if(UART_HandleStructure->pRxBuffPtr[3]==1){//kp
						HEX.pidPitch->setKp(HEX.remote->bytesToFloat(&UART_HandleStructure->pRxBuffPtr[4]));
						HEX.pidRoll->setKp(HEX.remote->bytesToFloat(&UART_HandleStructure->pRxBuffPtr[4]));
					}else if(UART_HandleStructure->pRxBuffPtr[3]==2){//ki
						HEX.pidPitch->setKi(HEX.remote->bytesToFloat(&UART_HandleStructure->pRxBuffPtr[4]));
						HEX.pidRoll->setKi(HEX.remote->bytesToFloat(&UART_HandleStructure->pRxBuffPtr[4]));
					}else if(UART_HandleStructure->pRxBuffPtr[3]==3){//kd
						HEX.pidPitch->setKd(HEX.remote->bytesToFloat(&UART_HandleStructure->pRxBuffPtr[4]));
						HEX.pidRoll->setKd(HEX.remote->bytesToFloat(&UART_HandleStructure->pRxBuffPtr[4]));
						for(int i = 0; i < 1; i++);
					}
				}
		}else if(UART_HandleStructure->pRxBuffPtr[0] == 255 && UART_HandleStructure->pRxBuffPtr[1] == 221){ // set IMU transmission frequency
			HEX.IMUTransFreq = HEX.remote->bytesToFloat(&UART_HandleStructure->pRxBuffPtr[2]);
		}

	}else if(UART_HandleStructure->Instance == UART3){							//accelerometer data

		if(UART_HandleStructure->pRxBuffPtr[0] == 255 && UART_HandleStructure->pRxBuffPtr[1]){	//ping command

		}else{

			HEX.accel->processAccelData();

			if(HEX.pidPitch->enabled || HEX.pidRoll->enabled){

				double pitch = HEX.pidPitch->calculate(HEX.pitchSetPoint, HEX.accel->pitch);	//if disabled it returns the set point (zero in most cases).
				double roll = HEX.pidRoll->calculate(HEX.rollSetPoint, HEX.accel->roll);

				double angleLimit = 0.6; //34.3775 degrees

				if( (pitch < angleLimit) && (pitch > -angleLimit) && (roll < angleLimit) && (roll > -angleLimit) ){

					HEX.setPose(roll, pitch);
					if(HEX.direction == -10){
						HEX.newPosition = true;
					}

				}else{

					if((pitch > angleLimit) || (pitch < -angleLimit)){
						//HEX.pidPitch->reset();
						//HEX.pidPitch->setParameters(-1.05, -2.2, 0);
					}
					if((roll > angleLimit) || (roll < -angleLimit)){
						//HEX.pidRoll->reset();
						//HEX.pidRoll->setParameters(-1.05, -2.2, 0);
					}

					if(HEX.direction == -10){
						//HEX.newPosition = true;
					}

				}

			}

			/*if(HEX.pidYaw->enabled){

				double yaw = HEX.pidYaw->calculate(HEX.yawSetPoint, HEX.accel->yaw);

				if(yaw <= 1 && yaw >= -1){
					//HEX.twistBody(yaw);
				}else{
					for(int i = 0; (i*i) < (yaw*yaw)*2; i++){
						//HEX.turnBody(yaw < 0 ? -0.5 : 0.5, ABSOLUTE);
					}
					//HEX.twistBody(0);
				}
			}*/
		}
	}
}

/**
 * @brief UART Error Handler
 *
 * @details
 *
 * @param UART_HandleStructure Pointer to the UART struct which caused the error interrupt.
 **/
void HAL_UART_ErrorCallback(UART_HandleTypeDef * UART_HandleStructure){
	uint32_t error = 0;
	uint8_t errorLoc = 0;

	__HAL_UNLOCK(UART_HandleStructure);

	__HAL_UART_CLEAR_FLAG(UART_HandleStructure, UART_FLAG_RXNE);
	__HAL_UART_CLEAR_FLAG(UART_HandleStructure, UART_FLAG_TC);

	if(UART_HandleStructure->Instance == UART1){
		//trace_printf("UART ERROR (%s): ", "UART1");
		errorLoc = 1;
	}else if(UART_HandleStructure->Instance == UART2){
		//trace_printf("UART ERROR (%s): ", "UART2");
		errorLoc = 2;
	}else if(UART_HandleStructure->Instance == UART3){
		//trace_printf("UART ERROR (%s): ", "UART3");
		errorLoc = 3;
	}

	switch(UART_HandleStructure->ErrorCode){
		case HAL_UART_ERROR_NONE:
			break;
		case HAL_UART_ERROR_DMA:
			error = HAL_UART_ERROR_DMA;
			//trace_printf("DMA transfer error");
			break;
		case HAL_UART_ERROR_FE:
			error = HAL_UART_ERROR_FE;
			__HAL_UART_CLEAR_FEFLAG(UART_HandleStructure);
			//trace_printf("Frame error");
			break;
		case HAL_UART_ERROR_NE:
			error = HAL_UART_ERROR_NE;
			__HAL_UART_CLEAR_NEFLAG(UART_HandleStructure);
			//trace_printf("Noise error");
			break;
		case HAL_UART_ERROR_ORE:
			error = HAL_UART_ERROR_ORE;
			__HAL_UART_CLEAR_OREFLAG(UART_HandleStructure);
			//trace_printf("Overrun error");
			break;
		case HAL_UART_ERROR_PE:
			error = HAL_UART_ERROR_PE;
			__HAL_UART_CLEAR_PEFLAG(UART_HandleStructure);
			//trace_printf("Parity error");
			break;
	}

	//trace_printf("\n");

	__HAL_UART_FLUSH_DRREGISTER(UART_HandleStructure);
	UART_HandleStructure->ErrorCode = 0;

	//__HAL_DMA_ENABLE(UART_HandleStructure->hdmarx);
	//__HAL_DMA_ENABLE(UART_HandleStructure->hdmatx);
	__HAL_UART_ENABLE(UART_HandleStructure);

	UART_HandleStructure->gState = HAL_UART_STATE_READY;
	UART_HandleStructure->RxState = HAL_UART_STATE_READY;

	if(errorLoc == 1){
		HEX.direction = -10;
		HEX.newPosition = false;
	}else if(errorLoc == 2 && !HEX.lowPowerMode){	//remote error
		HEX.remote->receiveRemoteData(10);
	}

	HEX.remote->sending = false;
	if(!HEX.lowPowerMode){
		HEX.remote->sendError(errorLoc, error);
	}
	HEX.remote->sending = false;
	//HEX._Error_Handler(__FILE__, __LINE__);		//hangs in infinite while loop
}

/** UNUSED IN DMA MODE, but needed or else code hangs **/
/**
 * @brief UART1 (Motor) Handler.
 *
 * @details
 **/
void USART1_IRQHandler(void){HAL_UART_IRQHandler(HEX.comms.getMotorUART());}

/**
 * @brief UART2 (Remote) Handler.
 *
 * @details
 **/
void USART2_IRQHandler(void){
	HAL_UART_IRQHandler(HEX.comms.getRemoteUART());
}

/**
 * @brief UART3 (IMU) Handler.
 *
 * @details
 **/
void USART3_IRQHandler(void){HAL_UART_IRQHandler(HEX.comms.getAccUART());}
/*********/

/** DMA INTERRUPTS (Runs before read/write) **/
/**
 * @brief Motor (UART1) TX DMA Handler.
 *
 * @details
 **/
void DMA2_Stream7_IRQHandler(void){
	HAL_DMA_IRQHandler(HEX.comms.getMotorDMA(TX));}

/**
 * @brief Motor (UART1) RX DMA Handler.
 *
 * @details
 **/
void DMA2_Stream2_IRQHandler(void){
	HAL_DMA_IRQHandler(HEX.comms.getMotorDMA(RX));}

/**
 * @brief Remote (UART2) TX DMA Handler.
 *
 * @details
 **/
void DMA1_Stream6_IRQHandler(void){
	HAL_DMA_IRQHandler(HEX.comms.getRemoteDMA(TX));
}

/**
 * @brief Remote (UART2) RX DMA Handler.
 *
 * @details
 **/
void DMA1_Stream5_IRQHandler(void){
	HAL_DMA_IRQHandler(HEX.comms.getRemoteDMA(RX));}

/**
 * @brief IMU (UART3) TX DMA Handler.
 *
 * @details
 **/
void DMA1_Stream3_IRQHandler(void){
	HAL_DMA_IRQHandler(HEX.comms.getAccDMA(TX));}

/**
 * @brief IMU (UART3) RX DMA Handler.
 *
 * @details
 **/
void DMA1_Stream1_IRQHandler(void){
	HAL_DMA_IRQHandler(HEX.comms.getAccDMA(RX));}
/*********/

/**
 * @brief Systick Handler.
 *
 * @details
 **/
void SysTick_Handler(void) {
	HAL_IncTick();
}

#ifdef __cplusplus
}
#endif //EXTERN "C"
