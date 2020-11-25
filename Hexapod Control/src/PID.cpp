/**
 * @file PID.cpp
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief
 **/

#include "PID.h"

/**
 * Implementation
 */
PID::PID(double dt, double Kp, double Ki, double Kd  ) :
    _pre_error(0),
    _integral(0)
{
	this->_dt = dt;
	this->_Kp = Kp;
	this->_Ki = Ki;
	this->_Kd = Kd;
	this->enabled = true;
}

PID::PID(){

}

PID::PID(double dt){
	this->_dt = dt;
}

PID::~PID(){

}

void PID::setParameters(double Kp, double Ki, double Kd){
	this->_Kp = Kp;
	this->_Ki = Ki;
	this->_Kd = Kd;
}

void PID::setKp(double kp){
	this->_Kp = kp;
	this->reset();
}

void PID::setKi(double ki){
	this->_Ki = ki;
	this->reset();
}

void PID::setKd(double kd){
	this->_Kd = kd;
	this->reset();
}


void PID::reset(){
	_pre_error = 0;
	_integral = 0;
}

double PID::calculate( double setpoint, double pv ){

	if(this->enabled){
		// Calculate error
		double error = pv - setpoint;

		// Proportional term
		double Pout = _Kp * error;

		// Integral term
		_integral += error * _dt;
		double Iout = _Ki * _integral;

		// Derivative term
		double derivative = (error-_pre_error)/_dt;
		double Dout = _Kd * derivative;

		// Calculate total output
		double output = Pout + Iout + Dout;

		// Save error to previous error
		_pre_error = error;

		return output;
	}else{
		return setpoint;
	}
}

double PID::getError(){
	return _pre_error;
}

void PID::togglePID(bool state){
	this->enabled = state;
}
