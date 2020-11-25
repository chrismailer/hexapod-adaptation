/**
 * @file Hexapod.cpp
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief Hexapod class functions.
 **/

#include "Hexapod.h"

Hexapod::Hexapod(void){

}

Hexapod::~Hexapod(void){

}

void Hexapod::Initialise(){

	for(int i = 0; i < 1000000;i++);	//delay (HAL_Delay causes timer issues)

	HAL_Init();
	this->SystemClock_Config();

	//initialise legs
	this->legs[0] = new Leg(2,	3,	4,	1, 	&this->comms);
	this->legs[1] = new Leg(5,	6,	7,	2, 	&this->comms);
	this->legs[2] = new Leg(8,	9,	10,	3, 	&this->comms);
	this->legs[3] = new Leg(11,	12,	13,	4,	&this->comms);
	this->legs[4] = new Leg(14,	15,	16,	5, 	&this->comms);
	this->legs[5] = new Leg(17,	18,	19,	6, 	&this->comms);

	this->setLegOffsets();

	this->remote = new Remote(&comms);	//initialise remote comms object
	this->accel = new Accelerometer(&comms);

	this->setTick();

	this->GPIO_init();
	this->ADC_init();

	this->disableMotors();

	/** INIT COMMS */
	comms.DMA_init();
	comms.UART_init();

	TIM2_init(270);			//motor control	300ms
	TIM3_init(100);			//accelerometer read data	100ms

	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);

	this->checkRemoteConfig();

	this->enableMotors();

	//this->resetAllMotors();	// wont work if motors aren't enabled


	/** SET INITIAL STANDING POSITION **/
	//for(int i = 0; i < 100000;i++);	//delay (HAL_Delay causes timer issues)
	//this->stand();
	//for(int i = 0; i < 100000;i++);	//delay (HAL_Delay causes timer issues)

	// Set gait controller frequency
	double dt = this->gaitPeriod / this->gaitFrequency;
	this->setTIM2Time(dt * 2000);

	//this->newPosition = true;
	/****/

	/** PID CONTROLLER FOR ACCELEROMETER **/
	this->pidPitch = new PID(this->getTIM(TIM3)->Init.Period / 1000.0);
	this->pidRoll = new PID(this->getTIM(TIM3)->Init.Period / 1000.0);
	this->pidYaw = new PID(this->getTIM(TIM3)->Init.Period / 1000.0);

	// reset Yaw to zero.
	this->accel->resetHeading(true);

	//for(int i = 0; i < 100000;i++);
	/** RECEIVE INTERRUPTS **/
	this->remote->receiveRemoteData(10);
	this->accel->receiveAccelData();

	//while(!this->PingAll());			//hang if any motor is not pinged successfully (motor not found)
}

void Hexapod::setLegOffsets(){
	/** LEG ALIGNMENT OFFSETS **/
	this->legs[0]->setOffsets(0, 0.121013860676953, 0.0153397851562342);
	this->legs[1]->setOffsets(0, 0, 0.0102265234374891);
	this->legs[2]->setOffsets(0, 0.100560813801975, 0.00511326171874504);
	this->legs[3]->setOffsets(0, 0.105674075520719, 0.0136353645833193);
	this->legs[4]->setOffsets(0, 0.156806692708165, 0);
	this->legs[5]->setOffsets(0, 0.0204530468749783, 0.0255663085937225);
}

void Hexapod::_Error_Handler(char * file, int line)
{
	trace_printf("ERROR in: %s at line number: %i\n", file, line);
	while(1);
}

void Hexapod::SystemClock_Config(void){
	  RCC_OscInitTypeDef RCC_OscInitStruct;
	  RCC_ClkInitTypeDef RCC_ClkInitStruct;

	  /* Enable Power Control clock */
	  __HAL_RCC_PWR_CLK_ENABLE();

	  /* The voltage scaling allows optimizing the power consumption when the device is
	     clocked below the maximum system frequency, to update the voltage scaling value
	     regarding system frequency refer to product datasheet.  */
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	  /* Configure RCC Oscillators: All parameters can be changed according to user's needs */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = 8;
	  RCC_OscInitStruct.PLL.PLLN = 336;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStruct.PLL.PLLQ = 7;
	  HAL_RCC_OscConfig (&RCC_OscInitStruct);

	  /* RCC Clocks: All parameters can be changed according to user�s needs */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK |RCC_CLOCKTYPE_HCLK |RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

	  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);			//set to interrupt on ms

	  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	  /* SysTick_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

bool Hexapod::ADC_init(){

	bool output = false;

	   /* __HAL_RCC_ADC1_CLK_ENABLE();

	    // ADC1_IN16 for internal temp sensor
	    ADC->CCR &= ~ADC_CCR_TSVREFE;		//wake up temp sensor

	    ADC_HandleStructure.Instance = ADC1;

	    ADC_HandleStructure.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	    ADC_HandleStructure.Init.Resolution = ADC_RESOLUTION_12B;
	    ADC_HandleStructure.Init.ScanConvMode = DISABLE;
	    ADC_HandleStructure.Init.ContinuousConvMode = ENABLE;
	    ADC_HandleStructure.Init.DiscontinuousConvMode = DISABLE;
	    ADC_HandleStructure.Init.NbrOfDiscConversion = 1;
	    ADC_HandleStructure.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	    ADC_HandleStructure.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	    ADC_HandleStructure.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	    ADC_HandleStructure.Init.NbrOfConversion = 1;
	    ADC_HandleStructure.Init.DMAContinuousRequests = DISABLE;
	    ADC_HandleStructure.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	    HAL_ADC_Init(&ADC_HandleStructure);

	    HAL_NVIC_SetPriority(ADC_IRQn, 3, 0);
	    HAL_NVIC_EnableIRQ(ADC_IRQn);

	    ADC_ChannelConfTypeDef adcChannel;

	    adcChannel.Channel = ADC_CHANNEL_TEMPSENSOR;
	    adcChannel.Rank = 1;
	    adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;

	    if (HAL_ADC_ConfigChannel(&ADC_HandleStructure, &adcChannel) == HAL_OK){
	    	return true;
	    }else{
	    	return false;
	    }*/

		/** CHARGING CIRCUIT ADC **/
	    __ADC1_CLK_ENABLE();

	    ADC_HandleStructure_Charger.Instance = ADC1;
	    ADC_HandleStructure_Charger.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	    ADC_HandleStructure_Charger.Init.Resolution = ADC_RESOLUTION_12B;
	    ADC_HandleStructure_Charger.Init.ScanConvMode = DISABLE;
	    ADC_HandleStructure_Charger.Init.ContinuousConvMode = ENABLE;
	    ADC_HandleStructure_Charger.Init.DiscontinuousConvMode = DISABLE;
	    ADC_HandleStructure_Charger.Init.NbrOfDiscConversion = 0;
	    ADC_HandleStructure_Charger.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	    ADC_HandleStructure_Charger.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	    ADC_HandleStructure_Charger.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	    ADC_HandleStructure_Charger.Init.NbrOfConversion = 1;
	    ADC_HandleStructure_Charger.Init.DMAContinuousRequests = DISABLE;
	    ADC_HandleStructure_Charger.Init.EOCSelection = DISABLE;

	    HAL_ADC_Init(&ADC_HandleStructure_Charger);

	    HAL_NVIC_SetPriority(ADC_IRQn, 3, 0);
	    HAL_NVIC_EnableIRQ(ADC_IRQn);

	    ADC_ChannelConfTypeDef adcChannel;

	    adcChannel.Channel = ADC_CHANNEL_5;
	    adcChannel.Rank = 1;
	    adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;

	    if (HAL_ADC_ConfigChannel(&ADC_HandleStructure_Charger, &adcChannel) == HAL_OK){
	    	output = true;
	    }else{
	    	return false;
	    }
	    /*****/

	    return output;

}

void Hexapod::enterLowPowerMode(void){

	this->lowPowerMode = true;

	this->disableMotors();

	__GPIOB_CLK_DISABLE();
	__GPIOD_CLK_DISABLE();

	HAL_TIM_Base_Stop_IT(&TIM2_HandleStructure);
	__TIM2_CLK_DISABLE();

	HAL_NVIC_DisableIRQ(DMA2_Stream7_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
	__HAL_UART_DISABLE_IT(comms.getMotorUART(), UART_IT_TXE);
	__DMA2_CLK_DISABLE();

	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(USART3_IRQn);

	__USART1_CLK_ENABLE();
	__USART3_CLK_ENABLE();
}


void Hexapod::exitLowPowerMode(void){
	this->lowPowerMode = false;

	this->Initialise();

	this->disableMotors();
}

bool Hexapod::GPIO_init(void){

	/** UART RELATED PINS **/
	__GPIOB_CLK_ENABLE();
	__GPIOD_CLK_ENABLE();

	// Configure pin in output push/pull mode					// choose pin for flow control
	GPIO_InitStructure.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;						//PD12 for flow control, rest LEDS
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;		//for USART

	GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7;			//USART1 ( TX|RX = PB6|PB7 )
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_5 | GPIO_PIN_6;			//USART2 ( TX|RX = PD5|PD6 )
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_9;			//USART3 ( TX|RX = PD8|PD9 )
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART3;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	/*****/

	/** CHARGE CIRCUIT PINS **/
	__GPIOA_CLK_ENABLE();

	// Configure pin in input analog
	GPIO_InitStructure.Pin = GPIO_PIN_4;	//enable charger (output)
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure pin in input analog
	GPIO_InitStructure.Pin = GPIO_PIN_5;	//ADC Voltage Input
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure pin in input	//pull down 6, pull up 7 to default FAULT status (eg if charger not connected)
	GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7;	//Status Input
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*****/

	/** MOTOR SWITCH PINS **/
	// Configure pin in output
	/*GPIO_InitStructure.Pin = GPIO_PIN_10;	//enable motors (output)
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);*/
	/*****/

	/** GPIO EXPANSION PINS **/
	__GPIOE_CLK_ENABLE();	//EXPANSION PORT

	// Configure pin in input analog
	GPIO_InitStructure.Pin = GPIO_PIN_8;// | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;	//expansion pins
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

	/*****/

	return true;
}

void Hexapod::TIM2_init(uint32_t time){	//time in miliseconds

	__TIM2_CLK_ENABLE();

	TIM2_HandleStructure.Instance = TIM2;
	TIM2_HandleStructure.Init.Prescaler = 42000;
	TIM2_HandleStructure.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM2_HandleStructure.Init.Period = time;
	TIM2_HandleStructure.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	if(HAL_TIM_Base_Init(&TIM2_HandleStructure) != HAL_OK){
		_Error_Handler((char *)__FILE__, __LINE__);
	}

	HAL_NVIC_SetPriority(TIM2_IRQn,2,0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void Hexapod::TIM3_init(uint32_t time){	//time in miliseconds

	__TIM3_CLK_ENABLE();

	TIM3_HandleStructure.Instance = TIM3;
	TIM3_HandleStructure.Init.Prescaler = 42000;
	TIM3_HandleStructure.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM3_HandleStructure.Init.Period = time;
	TIM3_HandleStructure.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	if(HAL_TIM_Base_Init(&TIM3_HandleStructure) != HAL_OK){
		_Error_Handler((char *)__FILE__, __LINE__);
	}

	HAL_NVIC_SetPriority(TIM3_IRQn,2,1);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void Hexapod::startTimers(){
	if(HAL_TIM_Base_Start_IT(&TIM2_HandleStructure) != HAL_OK){
		_Error_Handler((char *)__FILE__, __LINE__);
	}
	if(HAL_TIM_Base_Start_IT(&TIM3_HandleStructure) != HAL_OK){
		_Error_Handler((char *)__FILE__, __LINE__);
	}
}

void Hexapod::setTIM2Time(uint32_t time){
	__HAL_TIM_SET_COUNTER(&TIM2_HandleStructure, 0);
	__HAL_TIM_SET_AUTORELOAD(&TIM2_HandleStructure, (uint16_t)time);
}

void Hexapod::setTick(void){
	this->utick = HAL_GetTick();
}

uint32_t Hexapod::getTick(void){
	uint32_t time = HAL_GetTick() - this->utick;
	this->utick = HAL_GetTick();
	return time;
}

void Hexapod::readAllMotors(void){
	for(int i = 0; i < 6; i++){
		for(int j = 0; j < 3; j++){
			this->legs[i]->motors[j]->readReg(MODEL_NUMBER_L,50);
		}
	}
}

TIM_HandleTypeDef * Hexapod::getTIM(TIM_TypeDef * timNum){

	TIM_HandleTypeDef * value = NULL;


	if(timNum == TIM1){
		//value = &TIM1_HandleStructure;
	}else if(timNum == TIM2){

		value = &TIM2_HandleStructure;
	}if(timNum == TIM3){

		value = &TIM3_HandleStructure;
	}


	return value;

}

double Hexapod::readInternalTemp(){
	/*HAL_ADC_Start(&ADC_HandleStructure);
	HAL_ADC_PollForConversion(&ADC_HandleStructure, HAL_MAX_DELAY);
	uint32_t value = HAL_ADC_GetValue(&ADC_HandleStructure);

	double slope = (110.0F - 30.0F)/(*ADC_TEMP_3V3_30C - *ADC_TEMP_3V3_110C);

	this->currentTemperature = (double)(slope*(*ADC_TEMP_3V3_30C - (float)(value)) + 30.0F);

	if(this->startingTemperature == -9999){
		this->startingTemperature = this->currentTemperature;
	}
*/
   	return this->currentTemperature;
}

void Hexapod::enableChargerReading(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void Hexapod::disableChargerReading(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void Hexapod::startCharging(){
	uint8_t data[] = {255, 248, START_CHARGE, 0, 0, 0, 0,0,0,0};
	long checksumTemp = 0;
	for(int i = 0; i < 9; i++){
		checksumTemp += data[i];
	}
	data[9] = (uint8_t)checksumTemp;
	for(int i = 0; i < 500000; i++);
	this->comms.transmitRemote(data,10,100, DMA);
	for(int i = 0; i < 500000; i++);

	this->disableMotors();
}

void Hexapod::stopCharging(){
	uint8_t data[] = {255, 248, STOP_CHARGE, 0, 0, 0, 0,0,0,0};
	long checksumTemp = 0;
	for(int i = 0; i < 9; i++){
		checksumTemp += data[i];
	}
	data[9] = (uint8_t)checksumTemp;
	for(int i = 0; i < 500000; i++);
	this->comms.transmitRemote(data,10,100, DMA);
	for(int i = 0; i < 500000; i++);
}

void Hexapod::disableMotors(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);	//disable motors
	for(int i = 0; i < 1500000; i++);
}

void Hexapod::enableMotors(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);	//enable motors
	for(int i = 0; i < 1500000; i++);
}

struct CHARGER Hexapod::readCharger(){

	struct CHARGER charger;
	charger.status = NOT_CHARGING;
	charger.voltage = 0;
	this->enableChargerReading();

	for(int i = 0; i < 5000; i++);

	HAL_ADC_Start(&ADC_HandleStructure_Charger);

	if(HAL_ADC_PollForConversion(&ADC_HandleStructure_Charger, 1000 == HAL_OK)){
		double adcValue = (double)HAL_ADC_GetValue(&ADC_HandleStructure_Charger);
		charger.voltage = (adcValue/4095.0)*3.30*7.98;
	} else {
		charger.voltage = 0;
	}

	this->disableChargerReading();

	volatile uint8_t status0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
	volatile uint8_t status1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);

	if(status0){
		if(status1){
			charger.status = NOT_CHARGING;
		}else{
			charger.status = FAULT;
		}
	}else{
		if(status1){
			charger.status = CHARGING_CC;
		}else{
			charger.status = CHARGING_CV;
		}
	}

	charger.voltage += 0;

	return charger;
}

void Hexapod::checkRemoteConfig(){

	uint8_t readConfig[] = {0xA6, 0x6A};
	uint8_t remoteConfigReceive[7] = {0,0,0,0,0,0,0};

	uint8_t correctRemoteConfig[7] = {0xA6, 0x01, 0x64, 0x03, 0x0A, 0x00, 0x02};		//for Hexapod one only, (ID = 2 is only difference)

	comms.transmitRemote(readConfig, 2, 1000, BLOCKING);
	comms.receiveRemote(remoteConfigReceive, 7, 1000, BLOCKING);

	bool updateConfig = true;

	if(remoteConfigReceive[0] == 0){
		this->setRemoteConfig();
	}else{
		for(int i = 0; i < 7; i++){
			if(remoteConfigReceive[i] == 55){
				checkRemoteConfig();
			}else{
				if(remoteConfigReceive[i] != correctRemoteConfig[i]){
					updateConfig = true;
					break;
				}else{
					updateConfig = false;
				}
			}
		}
	}

	if(updateConfig){
		this->setRemoteConfig();
	}
		/*if(this->setRemoteConfig()){
			//success
			comms.transmitRemote(readConfig, 2, 1000, DMA);
			uint8_t remoteConfigReceive2[7] = {0,0,0,0,0,0,0};
			comms.receiveRemote(remoteConfigReceive2, 7, 1000, DMA);
			for(int i = 0; i < 7; i++){
				if(remoteConfigReceive2[i] != correctRemoteConfig[i]){
					updateConfig = true;
					break;
				}else{
					updateConfig = false;
				}
			}
			while(0);
		}else{
			//failed
			//trace_printf("REMOTE CONFIG INCORRECT\n");
			//char msg[5000];
			//sprintf(msg,"REMOTE CONFIG FAILED");
			//this->remote->sendASCII(msg);
			//while(1);
		}*/
	//}
}

bool Hexapod::setRemoteConfig(){
	uint8_t setConfigbaud[] = {0xA3, 0x3A, 0x03};		//baud rate to 19200
	uint8_t setConfigChannel[] = {0xA7, 0x7A, 0x01};		//set channel to 1
	uint8_t setConfigID[] = {0xA9, 0x9A, 0x00, 0x02};	//set ID to 2
	uint8_t setConfigPower[] = {0xAB, 0xBA, 0x0A};		//set power to 10dbm

	uint8_t confirmationReceive[] = {0};
	comms.setRemoteBaud(9600);
	comms.transmitRemote(setConfigbaud, 3, 1000, BLOCKING);
	comms.setRemoteBaud(19200);
	comms.receiveRemote(confirmationReceive, 1, 1000, BLOCKING);
	if(confirmationReceive[0] == 0xAA){
		confirmationReceive[0] = 0;

		comms.transmitRemote(setConfigChannel, 3, 1000, BLOCKING);
		comms.receiveRemote(confirmationReceive, 1, 1000, BLOCKING);

		if(confirmationReceive[0] == 0xAA){
			confirmationReceive[0] = 0;

			comms.transmitRemote(setConfigID, 4, 1000, BLOCKING);
			comms.receiveRemote(confirmationReceive, 1, 1000, BLOCKING);

			if(confirmationReceive[0] == 0xAA){
				confirmationReceive[0] = 0;

				comms.transmitRemote(setConfigPower, 3, 1000, BLOCKING);
				comms.receiveRemote(confirmationReceive, 1, 1000, BLOCKING);

				if(confirmationReceive[0] == 0xAA){
					//success
					return true;
				}else{
					//power set failed
					return false;
				}
			}else{
				//ID set failed
				return false;
			}
		}else{
			//channel set failed
			return false;
		}
	}else{
		//baud set failed
		return false;
	}
}



void Hexapod::setPIDSetPoints(double pitch, double roll){
	this->pitchSetPoint = pitch;
	this->rollSetPoint = roll;
}

void Hexapod::UART1_Toggle_Write(void){
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
}

void Hexapod::UART1_Toggle_Read(void){
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
}


void Hexapod::makePath(){
//	footVelocity, bodyHeight, crabAngle, gaitPeriod, legNumber, radialDistance, angleOffset, stepHeight, dutyFactor, slopePitch, slopeRoll
	for(int i=0; i<6; i++){
		this->legs[i]->makePath(footVelocity, bodyHeight, direction, gaitPeriod, radialDistances[i], angleOffsets[i], stepHeights[i], phaseOffsets[i], dutyFactors[i], slopePitch, slopeRoll);
	}
}

void Hexapod::setNextPathPoint(void){

	for(int i = 0; i < 6; i++){

		//if(!(this->liftedForBase && i == 0)){
			this->legs[i]->positionXYZ.p1 = this->legs[i]->legPath->pathXYZ[this->legs[i]->currentPathPoint].p1;
			this->legs[i]->positionXYZ.p2 = this->legs[i]->legPath->pathXYZ[this->legs[i]->currentPathPoint].p2;
			this->legs[i]->positionXYZ.p3 = this->legs[i]->legPath->pathXYZ[this->legs[i]->currentPathPoint].p3;

			this->legs[i]->inverseKinematics();
			this->legs[i]->setPositionBits();

			double xd = this->legs[i]->legPath->pathXYZ_dot[this->legs[i]->currentPathPoint].p1;//*this->speed*0.2;
			double yd = this->legs[i]->legPath->pathXYZ_dot[this->legs[i]->currentPathPoint].p2;//*this->speed*0.2;
			double zd = this->legs[i]->legPath->pathXYZ_dot[this->legs[i]->currentPathPoint].p3;//*this->speed*0.2;

			this->legs[i]->setSpeed(xd, yd, zd);
		//}
		int pathSize = (uint)sizeof(this->legs[i]->legPath->pathXYZ)/sizeof(this->legs[i]->legPath->pathXYZ[0]);
		if(this->legs[i]->currentPathPoint == pathSize-1){
			this->legs[i]->currentPathPoint = 0;
		}else{
			this->legs[i]->currentPathPoint++;
		}
	}
}

/*bool Hexapod::readyForNextPosition(void){

	uint16_t posError = 10;

	for(int i = 2; i < 20; i++){
		uint8_t * motor = this->getMotor(i)->controlTable;
		if(motor[8] == i){
			if( ((uint16_t)(motor[36] << 8 | motor[35]) - (uint16_t)(motor[42] << 8 | motor[41]))*((uint16_t)(motor[36] << 8 | motor[35]) - (uint16_t)(motor[42] << 8 | motor[41])) < posError*posError){
				this->atGoalPosition = true;
			}else{
				this->atGoalPosition = false;
			}
		}else{
			//trace_printf("NEXT POS ID ERROR\n");
			this->atGoalPosition = false;
		}
	}

	return this->atGoalPosition;

}*/

void Hexapod::setParamenters(double radialDistance, double strideLength, double stepHeight){
	this->radialDistance = radialDistance; 	//mm
//	this->strideLength = strideLength;		//mm
//	this->stepHeight = stepHeight;		//mm
}

void Hexapod::setDirection(double dir){
	this->direction = dir;
}

void Hexapod::setBodyHeight(double height){
	this->bodyHeight = height;
}

void Hexapod::stand(void){
	double stand[6][3] = {{283.710,0.0,-this->bodyHeight},{141.855,-245.7,-this->bodyHeight},{-141.855,-245.7,-this->bodyHeight},{-283.710,0.0,-this->bodyHeight},{-141.855,245.7,-this->bodyHeight},{141.855,245.7,-this->bodyHeight}};
	this->setPosition(stand);
	this->twistBody(0);
	this->setPose(0,0);
	this->setSpeedRPM(20,20,20);
	this->move(DMA);
	this->newPosition = false;
}

void Hexapod::setSpeedRPM(double th1d, double th2d, double th3d){
	for(int i = 0; i < 6; i++){
		this->legs[i]->setSpeedRPM(th1d, th2d, th3d);
	}
}

void Hexapod::update(void){
	// if there is data from the remote
	if(this->remote->newData){

		if(this->remote->charging){
				this->charging = true;
		}else{

			if(this->charging){
				this->charging = false;
			}

			// Body height changed
			if(this->remote->bodyHeight >= 50 && this->remote->bodyHeight <= 240){
				if((this->bodyHeight != this->remote->bodyHeight) && (this->direction == -10)){
					this->setBodyHeight(this->remote->bodyHeight);
					double * currentPos[6] = {this->legs[0]->positionXYZ.toArray(), this->legs[1]->positionXYZ.toArray(), this->legs[2]->positionXYZ.toArray(), this->legs[3]->positionXYZ.toArray(), this->legs[4]->positionXYZ.toArray(), this->legs[5]->positionXYZ.toArray()};
					for(int i = 0; i < 6; i++){
						currentPos[i][2] = -this->bodyHeight;
					}
					this->setPosition(currentPos);
					if(this->liftedForBase){
					//	this->legs[0]->setTheta(0, 1, -1.5);
					}
					this->move(DMA);
				}else{
					this->setBodyHeight(this->remote->bodyHeight);
				}
			}

			bool makePath = false;

			// Speed changed
			if(this->remote->speed != this->speed && this->remote->speed >= 0 && this->remote->speed <= 100){	//4.5 max speed from multiplier in process remote data.
				this->speed = this->remote->speed;
				if(this->speed == 0){
					this->direction = -10;
				} else {

//					this->setTIM2Time(600*(1.1-this->speed));
//					for(int i = 0; i < 6; i++){
//						double tfNew = (double)(2.33333*(1.1-this->speed));
//						// need to change this
//						this->legs[i]->legPath->tf = tfNew;
//					}

					makePath = true;
				}
			}

			// Direction changed
			if(this->direction != this->remote->direction){
				this->setDirection(this->remote->direction);
			}

			if(this->direction > -5 && direction < 5){
//				this->stepHeight = this->bodyHeight/2.0;			//step height dependent on body height

				if(this->bodyHeight >= 200){
//					this->stepHeight = this->bodyHeight/2.0-60.0;			//step height dependent on body height
//					this->strideLength = (1/this->bodyHeight)*14000-30.0;	//stride length dependent on body height
				}

				makePath = true;
			}

			if(makePath){
				this->makePath();
			}

			// Body twist changed
			if(this->bodyTwist != this->remote->twistBody){
				this->twistBody(this->remote->twistBody);
				this->yawSetPoint = this->remote->twistBody;
			}

			// Button functions
			this->function = this->remote->function;

			switch(this->function){
				case 1:
					break;
				case 2:
					this->function = 0;
					if(this->direction == -10){
						this->stand();
						this->newPosition = true;
					}
					break;
				case 3:
					this->function = 0;
					/*if(this->pidYaw->enabled){
						this->pidYaw->reset();
						this->yawSetPoint += 0.6;
					}*/
					this->turnBody(-0.3, ABSOLUTE);
					break;
				case 4:
					this->function = 0;
					/*if(this->pidYaw->enabled){
						this->pidYaw->reset();
						this->yawSetPoint -= 0.6;
					}*/
					this->turnBody(0.3, ABSOLUTE);
					break;
				case 5:
					this->function = 0;
					if(!this->liftedForBase){
						this->liftOntoBase(true);
					}else{
						this->liftOntoBase(false);
					}
					break;
				case 6:
					this->function = 0;
					break;
				case 7:
					this->function = 0;
					break;
				case 8:
					this->function = 0;
					break;
				case 55:
					this->fullSystemReset(true);
					break;
				default:
					this->function = 0;
					break;
			}
		}
	}
}

void Hexapod::setPose(double roll, double pitch){
	this->slopeRoll = -roll;
	this->slopePitch = pitch;
	this->legs[0]->setTiltOffset(this->legs[0]->positionXYZ.p2*tan(roll)+this->legs[0]->positionXYZ.p1*tan(pitch));
	this->legs[1]->setTiltOffset(this->legs[1]->positionXYZ.p2*tan(roll)+this->legs[1]->positionXYZ.p1*tan(pitch));
	this->legs[2]->setTiltOffset(this->legs[2]->positionXYZ.p2*tan(roll)+this->legs[2]->positionXYZ.p1*tan(pitch));
	this->legs[3]->setTiltOffset(this->legs[3]->positionXYZ.p2*tan(roll)+this->legs[3]->positionXYZ.p1*tan(pitch));
	this->legs[4]->setTiltOffset(this->legs[4]->positionXYZ.p2*tan(roll)+this->legs[4]->positionXYZ.p1*tan(pitch));
	this->legs[5]->setTiltOffset(this->legs[5]->positionXYZ.p2*tan(roll)+this->legs[5]->positionXYZ.p1*tan(pitch));
}

void Hexapod::twistBody(double angle){

	for(int i = 0; i < 6; i++){
		double newTheta1 = this->legs[i]->positionTHETA.p1 - this->bodyTwist + angle;
		this->legs[i]->setTheta(newTheta1,
								this->legs[i]->positionTHETA.p2,
								this->legs[i]->positionTHETA.p3);
		//this->legs[i]->setSpeedRPM(50,this->legs[i]->theta_dot->p2,this->legs[i]->theta_dot->p3);
	}

	this->bodyTwist = angle;
	this->newPosition = true;
}

void Hexapod::turnBody(double angle, enum TURN_MODE mode){

	UNUSED(mode);

	if(angle != 0.0){

		if(mode == ABSOLUTE){
			for(int i = 0; i < 6; i++){
				this->legs[i]->setTheta(0,this->legs[i]->positionTHETA.p2, this->legs[i]->positionTHETA.p3);
			}
		}

		for(int i = 0; i < 3; i++){

			this->legs[i*2+1]->setPosition(this->legs[i*2+1]->positionXYZ.p1,					//legs lift up 25, 1,3,5
										   this->legs[i*2+1]->positionXYZ.p2,
										   this->legs[i*2+1]->positionXYZ.p3 + 15.0);

			this->legs[i*2+1]->setTheta(this->legs[i*2+1]->positionTHETA.p1 - angle,		//legs rotate, 1,3,5
										this->legs[i*2+1]->positionTHETA.p2,
										this->legs[i*2+1]->positionTHETA.p3);

			this->legs[i*2]->setTheta(this->legs[i*2]->positionTHETA.p1 + angle,	//support legs rotate, 0,2,4
									  this->legs[i*2]->positionTHETA.p2,
									  this->legs[i*2]->positionTHETA.p3);
		}

		this->move(DMA);

		int delayTime = 4000000;

		for(int i = 0; i < delayTime;i++);

		for(int j = 0; j < 3; j++){

			this->legs[j*2]->setTheta(this->legs[j*2]->positionTHETA.p1 - angle,		//legs rotate, 0,2,4
									  this->legs[j*2]->positionTHETA.p2,
									  this->legs[j*2]->positionTHETA.p3);

			this->legs[j*2+1]->setTheta(this->legs[j*2+1]->positionTHETA.p1 + angle,	//support legs rotate, 1,3,5
										this->legs[j*2+1]->positionTHETA.p2,
										this->legs[j*2+1]->positionTHETA.p3);

			this->legs[j*2+1]->setPosition(this->legs[j*2+1]->positionXYZ.p1,					//legs back down
										   this->legs[j*2+1]->positionXYZ.p2,
										   this->legs[j*2+1]->positionXYZ.p3 - 30.0);

		}

		this->move(DMA);
		//HAL_Delay(400);
		for(int i = 0; i < delayTime;i++);

		for(int i = 0; i < 3; i++){	//legs up 25
			this->legs[i*2+1]->setPosition( this->legs[i*2+1]->positionXYZ.p1,
									this->legs[i*2+1]->positionXYZ.p2,
										this->legs[i*2+1]->positionXYZ.p3+15.0);

		}

		this->move(DMA);
	}
}

bool Hexapod::PingAll(){
	bool allMotorsPresent = false;
	for(int i = 2; i < 20; i++){
			allMotorsPresent = this->getMotor(i)->pingBool();
			if(!allMotorsPresent){
				this->remote->sendError(11, this->getMotor(i)->ID);
			}
	}

	return allMotorsPresent;
}

void Hexapod::resetAllMotors(void){
	for(int i = 2; i < 20; i++){
		this->getMotor(i)->resetErrors();
	}
}

void Hexapod::writeRegAll(enum CONTROL_TABLE reg, uint8_t * data, uint8_t dataLength){
	for(int i = 0; i < 6; i++){
		for(int j = 0; j < 3; j++){
			this->legs[i]->motors[j]->writeReg(reg, data, dataLength);
		}
	}
}


Motor * Hexapod::getMotor(uint8_t ID){
	if(ID >= 2 && ID <= 19){
		return this->legs[this->getMotorLegNum(ID)]->motors[ID%3 == 2 ? 0 : ID%3 == 0 ? 1 : 2];
	}else{
		return nullptr;
	}
}

uint8_t Hexapod::getMotorLegNum(uint8_t motorID){
	if(motorID >= 2 && motorID <= 4){
		return 0;
	}else if(motorID >= 5 && motorID <= 7){
		return 1;
	}else if(motorID >= 8 && motorID <= 10){
		return 2;
	}else if(motorID >= 11 && motorID <= 13){
		return 3;
	}else if(motorID >= 14 && motorID <= 16){
		return 4;
	}else if(motorID >= 17 && motorID <= 19){
		return 5;
	}else{
		return 6;
	}
}

void Hexapod::setSpeed(double xd, double yd, double zd){
	this->xd = xd;
	this->yd = yd;
	this->zd = zd;

	for(int i = 0; i < 6; i++){
		this->legs[i]->setSpeed(xd, yd, zd);
	}
}

void Hexapod::setPosition(double * pos[6]){
	for(int i = 0; i < 6; i++){
		this->legs[i]->setPosition(pos[i]);
	}
}

void Hexapod::setPosition(double pos[6][3]){
	for(int i = 0; i < 6; i++){
		this->legs[i]->setPosition(pos[i]);
	}
}

void Hexapod::setOrientation(double pitch, double roll, double yaw){
	this->pitchAngle = pitch;
	this->rollAngle = roll;
	this->yawAngle = yaw;
}

void Hexapod::setPositionBits(Point<uint8_t> * pos){
	for(int i = 0; i < 6; i++){
		this->legs[i]->setPositionBits(pos[i].p1, pos[i].p2, pos[i].p3);
	}
}

void Hexapod::move(enum UART_TRANSMISSION_TYPE type){

//	if(reachablePosition()){
	if (true) {

		//sync write only
		uint8_t numMotors = 18;
		uint8_t dataLength = 4;
		uint8_t length = (dataLength + 1)*numMotors+4;
		uint8_t ID = 0xFE;		//broadcast ID
		uint8_t instruction = SYNC_WRITE;
		uint8_t reg_address = GOAL_POSITION_L;

		uint8_t data2[length+4];

		data2[0] = data2[1] = 0xFF;		//start bits
		data2[2] = ID;
		data2[3] = length;
		data2[4] = instruction;
		data2[5] = reg_address;
		data2[6] = dataLength;

		uint8_t index = 7;
		for(int l = 0; l < 6; l++){

			for(int i = 0; i < 3; i++){

				data2[index] = this->legs[l]->motors[i]->ID;
				index++;

				uint8_t * tempBits = this->legs[l]->motors[i]->getGoalMovementBits();
				for(int j = 0; j < 4; j++){
					data2[index] = tempBits[j];
					index++;
				}

			}

		}

		data2[length+3] = 0;
		for(int i = 2; i < length+3; i++){
			data2[length+3] += data2[i];
		}

		data2[length+3] = ~data2[length+3];

		//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);	//for testing reading is not done so no need to set this
		//for(int i = 0; i < 1000; i++);
		comms.transmitMotors(data2, sizeof(data2)/sizeof(data2[0]), 0, type);
		this->elapsedTime += (this->gaitPeriod / this->gaitFrequency);
	}else{
		//trace_printf("UNREACHABLE\n");
		char msg[100];
		sprintf(msg,"UNREACHABLE");
		this->remote->sendASCII(msg);
	}
}

void Hexapod::liftOntoBase(bool state){
	this->liftedForBase = state;
	if(state){
		this->pidPitch->togglePID(false);
		this->pidRoll->togglePID(false);
		this->pidYaw->togglePID(false);
		this->setBodyHeight(190);
		this->setPose(0,0);
		this->legs[0]->setOffsets(this->legs[0]->offsets.p1+0.3,this->legs[0]->offsets.p2,this->legs[0]->offsets.p3);
		this->legs[5]->setOffsets(this->legs[5]->offsets.p1-0.3,this->legs[5]->offsets.p2,this->legs[5]->offsets.p3);
		//this->legs[0]->setTheta(0, 1, -1.5);
		//this->legs[0]->setPosition(100,0.0,20);
	}else{
		//this->legs[0]->setPosition(283.710,0.0,-this->bodyHeight);
		this->setBodyHeight(140);
		this->legs[0]->setOffsets(this->legs[0]->offsets.p1-0.3,this->legs[0]->offsets.p2,this->legs[0]->offsets.p3);
		this->legs[5]->setOffsets(this->legs[5]->offsets.p1+0.3,this->legs[5]->offsets.p2,this->legs[5]->offsets.p3);
	}
	this->stand();
	//this->newPosition = true;
}

void Hexapod::stop(void){
	this->direction = -10;
	this->newPosition = false;
}

bool Hexapod::reachablePosition(void){

	double legTooCloseError = this->legs[0]->bodyRadius+this->legs[0]->L1x;									//must be greater than this
	legTooCloseError *= legTooCloseError;

	bool reachable = false;

	//120 degrees - acos(...);
	double legCrashTheta = 2.094395 - acos(this->legs[0]->bodyRadius/(2*(this->legs[0]->L1x+this->legs[0]->L2+this->legs[0]->L3)))-0.02;	//actual crash point plus a tolerance.

	for(int i = 0; i < 6; i++){	//check if each leg could reach the position (goal pos not too close to body or too far from leg)

		double legTooFarError = this->legs[0]->bodyRadius+this->legs[0]->L1x+this->legs[0]->L1z/cos(this->legs[i]->positionTHETA.p2)+this->legs[0]->L2+this->legs[0]->L3;	//must be less than this
		legTooFarError *= legTooFarError;

		double legTooFarPos = this->legs[i]->positionXYZ.p1*this->legs[i]->positionXYZ.p1 + this->legs[i]->positionXYZ.p2*this->legs[i]->positionXYZ.p2 + this->legs[i]->positionXYZ.p3*this->legs[i]->positionXYZ.p3;
		if(legTooFarPos <= legTooFarError){
			reachable = true;
		}else{
			return false;
		}

		double legTooClosePos = this->legs[i]->positionXYZ.p1*this->legs[i]->positionXYZ.p1 + this->legs[i]->positionXYZ.p2*this->legs[i]->positionXYZ.p2 + this->legs[i]->positionXYZ.p3*this->legs[i]->positionXYZ.p3;
		if(legTooClosePos >= legTooCloseError){
			reachable = true;
		}else{
			return false;
		}

		double multiplier = 1;
		if((this->legs[i]->positionTHETA.p1 <= 0 && this->legs[(i+1>5?0:i+1)]->positionTHETA.p1 <= 0) || (this->legs[i]->positionTHETA.p1 >= 0 && this->legs[(i+1>5?0:i+1)]->positionTHETA.p1 >= 0)){	//only if positioned towards each other
			multiplier *= 1.34;
		}else{

			double thsq_1 = this->legs[i]->positionTHETA.p1;
			double thsq_2 = this->legs[(i+1>5?0:i+1)]->positionTHETA.p1;

			double abs = Lib_abs(thsq_1) + Lib_abs(thsq_2);
			if(abs <= 2*legCrashTheta*multiplier){		//issues here
				reachable = true;
			}else{
				//return false;
			}
		}

	}

	return reachable;
}

double Hexapod::Lib_abs(double val){
	if(val < 0){
		return val*(-1);
	}else{
		return val;
	}
}

void Hexapod::fullSystemReset(bool check){
	if(check){
		HAL_NVIC_SystemReset();
	}
}
