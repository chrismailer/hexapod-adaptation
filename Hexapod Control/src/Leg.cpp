/**
 * @file Leg.cpp
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief Leg class functions.
 **/

#include "Leg.h"

Leg::Leg(CommsManager * comms){
	this->comms = comms;
	this->motors[0] = NULL;
	this->motors[1] = NULL;
	this->motors[2] = NULL;
}

Leg::Leg(Motor m1, Motor m2, Motor m3, uint16_t legNumber, CommsManager * comms){

	this->comms = comms;

	this->motors[0] = new Motor(m1);
	this->motors[1] = new Motor(m2);
	this->motors[2] = new Motor(m3);

	this->positionTHETA.p1 = 0;
	this->positionTHETA.p2 = 0;
	this->positionTHETA.p3 = 0;

	this->theta_dot->p1 = 0;
	this->theta_dot->p2 = 0;
	this->theta_dot->p3 = 0;

	this->positionXYZ.p1 = 0;
	this->positionXYZ.p2 = 0;
	this->positionXYZ.p3 = 0;

	this->legNumber = legNumber;
	this->legAngle = (legNumber-1.0)*(M_PI/3);
}

Leg::Leg(uint8_t ID1, uint8_t ID2, uint8_t ID3, uint16_t legNumber, CommsManager * comms){

	this->comms = comms;

	this->motors[0] = new Motor(ID1, comms);
	this->motors[1] = new Motor(ID2, comms);
	this->motors[2] = new Motor(ID3, comms);

	this->positionTHETA.p1 = 0;
	this->positionTHETA.p2 = 0;
	this->positionTHETA.p3 = 0;

	this->theta_dot->p1 = 0;
	this->theta_dot->p2 = 0;
	this->theta_dot->p3 = 0;

	this->positionXYZ.p1 = 0;
	this->positionXYZ.p2 = 0;
	this->positionXYZ.p3 = 0;

	this->legNumber = legNumber;
	this->legAngle = (legNumber-1.0)*(M_PI/3);
}


Leg::~Leg(void){

}

void Leg::setOffsets(double off1, double off2, double off3){
	this->offsets.p1 = off1;
	this->offsets.p2 = off2;
	this->offsets.p3 = off3;
}

void Leg::setPosition(double x, double y, double z){
	this->positionXYZ.p1 = x;
	this->positionXYZ.p2 = y;
	this->positionXYZ.p3 = z;

	this->inverseKinematics();
	this->setPositionBits();
}


void Leg::setPosition(double * xyz){
	this->positionXYZ.p1 = xyz[0];
	this->positionXYZ.p2 = xyz[1];
	this->positionXYZ.p3 = xyz[2];

	this->inverseKinematics();
	this->setPositionBits();

	//this->forwardKinematics();
}

void Leg::setTheta(double theta1, double theta2, double theta3){
	this->positionTHETA.p1 = theta1;
	this->positionTHETA.p2 = theta2;
	this->positionTHETA.p3 = theta3;

	this->forwardKinematics();
	this->setPositionBits();
}

void Leg::setTheta(double * theta){
	this->positionTHETA.p1 = theta[0];
	this->positionTHETA.p2 = theta[1];
	this->positionTHETA.p3 = theta[2];

	this->forwardKinematics();
	this->setPositionBits();
}


double Leg::Lib_abs(double val){
	if(val < 0){
		return val*(-1);
	}else{
		return val;
	}
}

void Leg::setSpeed(double _xd, double _yd, double _zd){

	double th1 = this->positionTHETA.p1;	//radians
	double th2 = this->positionTHETA.p2;
	double th3 = this->positionTHETA.p3;

	double xd_global = (_xd/1000.0);	//m/s		// global coords
	double yd_global = (_yd/1000.0);
	double zd_global = (_zd/1000.0);

	double xd = (xd_global)*cos(this->legAngle) - (yd_global+0.0)*sin(this->legAngle);
	double yd = (yd_global)*cos(this->legAngle) + (xd_global+0.0)*sin(this->legAngle);
	double zd = zd_global;

	/*if(th3 == 0){
		th3 = 0.1;
	}*/

	L1x *= 0.001;
	L1z *= 0.001;
	L2 *= 0.001;
	L3 *= 0.001;

	double _thd1 =  (yd*cos(th1) - xd*sin(th1))/(L1x + L3*cos(th2 + th3) + L2*cos(th2)) ;	//correct

	double _thd2 = (1/L2)*( zd*cos(th2) - xd*cos(th1)*sin(th2) - yd*sin(th1)*sin(th2) + ((cos(th3))/(sin(th3)))*(zd*sin(th2) + xd*cos(th1)*cos(th2) + yd*cos(th2)*sin(th1)) );// (zd*cos(th2))/L2 - (xd*cos(th1)*sin(th2))/L2 - (yd*sin(th1)*sin(th2))/L2 + (L3*L3*zd*cos(th3)*sin(th2)*sin(th3))/(- L2*L3*L3*cos(th3)*cos(th3) + L2*L3*L3 + L2 + L2*cos(th1)) + (L3*L3*xd*cos(th1)*cos(th2)*cos(th3)*sin(th3))/(- L2*L3*L3*cos(th3)*cos(th3) + L2*L3*L3 + L2 + L2*cos(th1)) + (L3*L3*yd*cos(th2)*cos(th3)*sin(th1)*sin(th3))/(- L2*L3*L3*cos(th3)*cos(th3) + L2*L3*L3 + L2 + L2*cos(th1));
			//(zd*sin(th2 + th3) + xd*cos(th2 + th3)*cos(th1) + yd*cos(th2 + th3)*sin(th1))/(L2*0.001*sin(th3));//old

	double _thd3 = -(1/L2)*( zd*cos(th2) - xd*cos(th1)*sin(th2) - yd*sin(th1)*sin(th2) + (L2 + L3*(cos(th3))/(L3*sin(th3)))*(zd*sin(th2) + xd*cos(th1)*cos(th2) + yd*cos(th2)*sin(th1)) );// (zd*cos(th2))/L2 - (xd*cos(th1)*sin(th2))/L2 - (yd*sin(th1)*sin(th2))/L2 + (L3*L3*zd*cos(th3)*sin(th2)*sin(th3))/(- L2*L3*L3*cos(th3)*cos(th3) + L2*L3*L3 + L2 + L2*cos(th1)) + (L3*L3*xd*cos(th1)*cos(th2)*cos(th3)*sin(th3))/(- L2*L3*L3*cos(th3)*cos(th3) + L2*L3*L3 + L2 + L2*cos(th1)) + (L3*L3*yd*cos(th2)*cos(th3)*sin(th1)*sin(th3))/(- L2*L3*L3*cos(th3)*cos(th3) + L2*L3*L3 + L2 + L2*cos(th1));;
	//-(zd*cos(th2) + L3*L3*zd*cos(th2) + zd*cos(th1)*cos(th2) - xd*cos(th1)*sin(th2) - yd*sin(th1)*sin(th2) - xd*cos(th1)*cos(th1)*sin(th2) - L3*L3*xd*cos(th1)*sin(th2) - L3*L3*yd*sin(th1)*sin(th2) - yd*cos(th1)*sin(th1)*sin(th2) - L3*L3*zd*cos(th2)*cos(th3)*cos(th3) + L3*L3*zd*cos(th3)*sin(th2)*sin(th3) + L2*L3*zd*sin(th2)*sin(th3) + L3*L3*xd*cos(th1)*cos(th3)*cos(th3)*sin(th2) + L3*L3*yd*cos(th3)*cos(th3)*sin(th1)*sin(th2) + L3*L3*yd*cos(th2)*cos(th3)*sin(th1)*sin(th3) + L2*L3*xd*cos(th1)*cos(th2)*sin(th3) + L2*L3*yd*cos(th2)*sin(th1)*sin(th3) + L3*L3*xd*cos(th1)*cos(th2)*cos(th3)*sin(th3))/(L2*(- L3*L3*cos(th3)*cos(th3) + L3*L3 + cos(th1) + 1));
			//-(zd*(L3*0.001*sin(th2 + th3) + L2*0.001*sin(th2)))/(L2*0.001*L3*0.001*sin(th3)) - (xd*cos(th1)*(L3*0.001*cos(th2 + th3) + L2*0.001*cos(th2)))/(L2*0.001*L3*0.001*sin(th3)) - (yd*sin(th1)*(L3*0.001*cos(th2 + th3) + L2*0.001*cos(th2)))/(L2*0.001*L3*0.001*sin(th3)) ;//old

	L1x /= 0.001;
	L1z /= 0.001;
	L2 /= 0.001;
	L3 /= 0.001;

	double thd1 = Lib_abs(_thd1)*9.549296596425384;	//convert from rad/s to RPM
	double thd2 = Lib_abs(_thd2)*9.549296596425384;
	double thd3 = Lib_abs(_thd3)*9.549296596425384;

	if(isnan(thd1)){
		this->theta_dot->p1 = 10;
	}else if(thd1 < 1){
		//this->theta_dot->p1 = 1;
	}else if(thd1 > 114){
		//this->theta_dot->p1 = 114;
	}else{
		this->theta_dot->p1 = thd1*1.2;
	}

	if(isnan(thd2)){
		this->theta_dot->p2 = 25;
	}else if(thd2 < 1){
		//this->theta_dot->p2 = 25;
	}else if(thd2 > 114){
		//this->theta_dot->p2 = 114;
	}else{
		this->theta_dot->p2 = thd2*1.4;
	}

	if(isnan(thd3)){
		this->theta_dot->p3 = 25;//15;
	}else if(thd3 < 1){
		//this->theta_dot->p3 = 25;
	}else if(thd3 > 114){
		//this->theta_dot->p3 = 114;
	}else{
		this->theta_dot->p3 = thd3;
	}

	// Overwriting servo sopeed control
	this->theta_dot->p1 = 75; // rpm
	this->theta_dot->p2 = 60; // rpm
	this->theta_dot->p3 = 75; // rpm

	this->setSpeedBits();
}

void Leg::setSpeedRPM(double th1d, double th2d, double th3d){
	this->theta_dot->p1 = th1d;
	this->theta_dot->p2 = th2d;
	this->theta_dot->p3 = th3d;

	this->setSpeedBits();
}

void Leg::setSpeed(double * theta_dot){
	this->theta_dot->p1 = theta_dot[0];
	this->theta_dot->p2 = theta_dot[1];
	this->theta_dot->p3 = theta_dot[2];

	this->setSpeedBits();
}

void Leg::setPositionBits(){
	this->motors[0]->calcPositionBits((double)(this->positionTHETA.p1 - this->offsets.p1));
	this->motors[1]->calcPositionBits((double)(this->positionTHETA.p2 - this->offsets.p2));
	this->motors[2]->calcPositionBits((double)(this->positionTHETA.p3 - this->offsets.p3));
}

void Leg::setSpeedBits(){
	this->motors[0]->calcSpeedBits((double)this->theta_dot->p1);
	this->motors[1]->calcSpeedBits((double)this->theta_dot->p2);
	this->motors[2]->calcSpeedBits((double)this->theta_dot->p3);
}

void Leg::setTiltOffset(double tiltOffset){
	//this->positionXYZ.p3 -= this->tiltOffset;
	this->tiltOffset = tiltOffset;
	//this->positionXYZ.p3 += this->tiltOffset;
	this->inverseKinematics();
	this->setPositionBits();
}


void Leg::inverseKinematics(void){

	double x = this->positionXYZ.p1;
	double y = this->positionXYZ.p2;
	double z = this->positionXYZ.p3 + this->tiltOffset;

	x = (this->positionXYZ.p1+0.0)*cos(this->legAngle) - (this->positionXYZ.p2+0.0)*sin(this->legAngle) - this->bodyRadius;
	y = (this->positionXYZ.p2+0.0)*cos(this->legAngle) + (this->positionXYZ.p1+0.0)*sin(this->legAngle);

	this->positionTHETA.p1 = atan2(y,x);

	const double C3_temp = (  (x - L1x*cos(this->positionTHETA.p1))*(x - L1x*cos(this->positionTHETA.p1)) + (y - L1x*sin(this->positionTHETA.p1))*(y - L1x*sin(this->positionTHETA.p1)) + (z - L1z)*(z - L1z) - L2*L2 - L3*L3  )/(  2*L2*L3  );
	const double C3 = (C3_temp>1 ? 1 : (C3_temp<-1 ? -1 : C3_temp));

	const double S3 = -sqrt( 1 - C3*C3 );

	this->positionTHETA.p2 = atan2(z - L1z, sqrt( (x - L1x*cos(this->positionTHETA.p1))*(x - L1x*cos(this->positionTHETA.p1)) + (y - L1x*sin(this->positionTHETA.p1))*(y - L1x*sin(this->positionTHETA.p1)) ) ) - atan2(L3*S3, L2 + L3*C3);

	this->positionTHETA.p3 = atan2(S3,C3);

//	this->positionTHETA.p1 += 0.0;
//	this->positionTHETA.p2 += 0.0;
//	this->positionTHETA.p3 += 0.0;

	// simulated leg failure
	if ((this->legNumber == 6) or (this->legNumber == 3)) {
		this->positionTHETA.p1 = 0.0;
		this->positionTHETA.p2 = (100.0/180.0)*M_PI;
		this->positionTHETA.p3 = (-400.0/180.0)*M_PI;
	}
}

void Leg::forwardKinematics(void){
	const double x_local = 0.0 + cos(this->positionTHETA.p1)*((L1x) + (L3)*cos(this->positionTHETA.p2 + this->positionTHETA.p3)+(L2)*cos(this->positionTHETA.p2));
	const double y_local = 0.0 + sin(this->positionTHETA.p1)*((L1x) + (L3)*cos(this->positionTHETA.p2 + this->positionTHETA.p3)+(L2)*cos(this->positionTHETA.p2));
	const double z_local = 0.0 + (L1z) + (L3)*sin(this->positionTHETA.p2 + this->positionTHETA.p3) + (L2)*sin(this->positionTHETA.p2);

	/*if(LEG == 0 || LEG == 180){
		//convert from local coordinates to global
		this->positionXYZ.p1 = 0.0 + x_local + this->bodyRadius;// + ( y_local  -  ( sin(LEG)*(-this->bodyRadius - x_local) + y_local*cos(LEG) )*cos(LEG) );///( sin(LEG) );
		this->positionXYZ.p2 = 0.0 +     		   ( sin(LEG)*(-this->bodyRadius - x_local) + y_local*cos(LEG) );
	}else{
		//convert from local coordinates to global
		this->positionXYZ.p1 = 0.0 + ( y_local  -  ( sin(LEG)*(-this->bodyRadius - x_local) + y_local*cos(LEG) )*cos(LEG) )/( sin(LEG) );
		this->positionXYZ.p2 = 0.0 +     		   ( sin(LEG)*(-this->bodyRadius - x_local) + y_local*cos(LEG) );
	}*/

	this->positionXYZ.p1 = y_local*sin(this->legAngle)+(x_local+this->bodyRadius)*cos(this->legAngle);	//local to global
	this->positionXYZ.p2 = y_local*cos(this->legAngle)-(x_local+this->bodyRadius)*sin(this->legAngle);

	this->positionXYZ.p3 = 0.0 + z_local;
}

void Leg::writeRegs(enum CONTROL_TABLE reg, uint8_t * data, uint8_t dataLength){
	//sync write only
	uint8_t length = (dataLength + 1)*3+4;

	uint8_t packet[length+4];

	packet[0] = packet[1] = 0xFF;
	packet[2] = 0xFE;
	packet[3] = length;
	packet[4] = SYNC_WRITE;
	packet[5] = reg;
	packet[6] = dataLength;

	for(int i = 0; i < 3; i++){					//all 3 motors
		packet[7+5*i] = this->motors[i]->ID;
		for(int j = 0; j < dataLength; j++){	// data length count of data
			packet[8+5*i+j] = data[j+4*i];
		}
	}

	packet[length+3] = 0;
	for(int i = 2; i < length+3; i++){
		packet[length+3] += packet[i];
	}

	packet[length+3] = ~packet[length+3];

	this->UARTWrite(packet, sizeof(packet)/sizeof(packet[0]));
}

void Leg::move(){
	uint8_t data[12];
	for(int i = 0, count = 0; i < 3; i++){
		uint8_t * temp = this->motors[i]->getGoalMovementBits();
		for(int j = 0; j < 4; j++){
			data[count] = temp[j];
			count++;
		}
	}

	this->writeRegs(GOAL_POSITION_L, data, 4);
}

void Leg::makePath(	double footVelocity,
					double bodyHeight,
					double crabAngle,
					double gaitPeriod,
					double radialDistance,
					double angleOffset,
					double stepHeight,
					double phaseOffset,
					double dutyFactor,
					double slopePitch,
					double slopeRoll){

	this->currentPathPoint = (int) this->legPath->pathSize * phaseOffset;
	this->legPath->setPoints(footVelocity, bodyHeight, crabAngle, gaitPeriod, this->legNumber, radialDistance, angleOffset, stepHeight, dutyFactor, slopePitch, slopeRoll);
}


Motor * Leg::getMotor(uint8_t ID){
	return this->motors[ID%3];
;}

void Leg::UARTWrite(uint8_t * data, uint8_t dataLength){
	uint64_t timeout = 0;
	while(this->comms->getMotorUART()->gState != HAL_UART_STATE_READY && timeout!=10000000){timeout++;}
	if(this->comms->getMotorUART()->gState == HAL_UART_STATE_READY){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
		comms->transmitMotors(data,dataLength,0,DMA);
	}
}
