/**
 * @file PID.h
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief
 **/
#ifndef INCLUDE_PID_H
#define INCLUDE_PID_H

#include "stm32f4xx.h"

/**
 * 	@brief PID Class
 *
 *	@details
 *
 **/
class PID{

public:
	 /** @brief The time interval of the controller.  **/
    double _dt = 0.05;
    /** @brief The proportional gain of the controller.  **/
    double _Kp = 1.08;			// was -1.5, all zero no osc at -1.3, was -1.05
    /** @brief The integral gain of the controller.  **/
    double _Ki = 2.2;	// was -2.2
    /** @brief The differential gain of the controller.  **/
    double _Kd =  0.027;
    /** @brief The error in the controller. **/
    double _pre_error = 0;
    /** @brief The cumulative integral of the controller.  **/
    double _integral = 0;

    /** @brief The state of the controller. **/
    bool enabled = false;

	/**
	 * @brief Default constructor for the class.
	 *
	 * @details Class constructor
	 *
	 **/
    PID();

	/**
	 * @brief Constructor for the class initialising dt, Kp, Ki and Kd.
	 *
	 * @details Class constructor
	 *
	 * @param dt The time interval of the controller.
	 * @param Kp The proportional gain of the controller.
	 * @param Ki The integral gain of the controller.
	 * @param Kd The differential gain of the controller.
	 **/
	PID(double dt, double Kp, double Ki, double Kd );

	/**
	 * @brief Constructor for the class initialising dt.
	 *
	 * @details Class constructor
	 *
	 * @param dt The time interval of the controller.
	 **/
    PID(double dt);

	/**
	 * @brief Default destructor for the class.
	 *
	 * @details Class destructor cleans up memory by deleting pointers and lists.
	 *
	 **/
    ~PID();

	/**
	 * @brief Set the PID controller gain parameters.
	 *
	 * @param Kp PID controller proportional gain.
	 * @param Ki PID controller integral gain.
	 * @param Kd PID controller derivative gain.
	 *
	 **/
    void setParameters(double Kp, double Ki, double Kd);

	/**
	 * @brief Set the PID controller gain Kp.
	 *
	 * @param kp PID controller proportional gain.
	 *
	 **/
    void setKp(double kp);

	/**
	 * @brief Set the PID controller gain Ki.
	 *
	 * @param ki PID controller integral gain.
	 *
	 **/
    void setKi(double ki);

	/**
	 * @brief Set the PID controller gain Kd.
	 *
	 * @param kd PID controller derivative gain.
	 *
	 **/
    void setKd(double kd);

	/**
	 * @brief Toggle the PID controller on or off.
	 *
	 * @details
	 *
	 **/
    void togglePID(bool state);

	/**
	 * @brief Reset the PID controller.
	 *
	 * @details
	 *
	 **/
    void reset();

	/**
	 * @brief Return the error term from the PID controller.
	 *
	 * @details
	 *
	 **/
    double getError();

	/**
	 * @brief Calculates the output of the PID controller.
	 *
	 * @details
	 *
	 * @param setpoint The set point (command) value for the PID controller.
	 * @param pv The value to compare the set point to (angle reading).
	 *
	 * @return The output of the PID controller.
	 *
	 **/
    double calculate( double setpoint, double pv );
};

#endif
