/**
 * @file Leg.h
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief Leg class
 **/

#ifndef INCLUDE_LEG_H
#define INCLUDE_LEG_H

#include "stm32f4xx.h"
#include "Math.h"
#include "Motor.h"
#include "Path.h"

#include "CommsManager.h"


/**
 * 	@brief Leg Class
 *
 *	@details
 *
 **/
class Leg{

private:
	/** @brief A pointer to the global communications manager. **/
	CommsManager * comms;

	/**
	 * 	@brief Write data to the UART channel of the leg.
	 *
	 *	@param data A pointer to the data to write.
	 *	@param dataLength The length of the data to write.
	 *
	 *	@details
	 **/
	void UARTWrite(uint8_t * data, uint8_t dataLength);

public:
	/** @brief A point array containing the default leg offsets.  **/
	Point<double> offsets;
	/** @brief The legs link 1 x dimension. **/
	double L1x = 53.17;
	/** @brief The legs link 1 z dimension. **/
//	double L1z = 8.0;
	double L1z = 0.0;
	/** @brief The legs link 2 dimension. **/
	double L2 = 101.88;
	/** @brief The legs link 3 dimension. **/
	double L3 = 149.16;						//was 152.0 with old foot
	/** @brief The hexapod's body radius. **/
	const double bodyRadius = 125.54;

	/** @brief The number of the leg. **/
	int legNumber;

	/** @brief The number of the leg. **/
	double legAngle;

	/** @brief A point array containing the RPM of each motor in the leg. **/
	Point<double> * theta_dot = new Point<double>(0,0,0,THETA);

	/** @brief A pointer to an array of motor objects. **/
	Motor * motors[3];
	/** @brief A Point object containing the current XYZ position of the leg. **/
	Point<double> positionXYZ;
	/** @brief A Point object containing the current theta values of the motors of the leg. **/
	Point<double> positionTHETA;
	/** @brief A counter holding the current position in the walking path. **/
	int currentPathPoint = 0;
	/** @brief A Path object holding the current path for the leg. **/
	Path * legPath = new Path();

	/** @brief The tilt offset of the hexapod relative to its current position. **/
	double tiltOffset = 0;

	/**
	 * @brief Default constructor for the class.
	 *
	 * @details Class constructor
	 **/
	Leg(CommsManager * comms);

	/**
	 * 	@brief Constructor for the class initializing leg parameters.
	 *
	 * 	@param m1 The first motor in the leg.
	 * 	@param m2 The second motor in the leg.
	 * 	@param m3 The third motor in the leg.
	 * 	@param legNumber The number of the leg.
	 * 	@param comms A pointer to the global communications manager.
	 *
	 *
	 * 	@details Class constructor
	 **/
	Leg(Motor m1, Motor m2, Motor m3, uint16_t legNumber, CommsManager * comms);

	/**
	 * 	@brief Constructor for the class initializing leg parameters.
	 *
	 *	@param ID1 The ID number of the first motor in the leg.
	 *	@param ID2 The ID number of the second motor in the leg.
	 *	@param ID3 The ID number of the third motor in the leg.
	 *	@param legNumber The number of the leg.
	 * 	@param comms A pointer to the global communications manager.
	 *
	 * 	@details Class constructor
	 *
	 **/
	Leg(uint8_t ID1, uint8_t ID2, uint8_t ID3, uint16_t legNumber, CommsManager * comms);

	/**
	 * @brief Default destructor for the class.
	 *
	 * @details Class destructor cleans up memory by deleting pointers and lists.
	 *
	 **/
	~Leg(void);

	/**
	 * 	@brief Calculates the inverse kinematics of the leg.
	 *
	 *	@details
	 **/
	void inverseKinematics(void);

	/**
	 * 	@brief Calculate the forward kinematics of the leg.
	 *
	 *	@details
	 **/
	void forwardKinematics(void);

	/**
	 * 	@brief Get the absolute value of a number.
	 *
	 *	@details
	 *
	 *	@param val The value to get the absolute of.
	 *
	 *	@return The absolute value.
	 **/
	double Lib_abs(double val);


	/**
	 * 	@brief Get the motor object corresponding to the motor ID
	 *
	 *	@details
	 *
	 *	@param ID The ID of the motor to return.
	 *
	 *	@return A pointer to the motor object.
	 **/
	Motor * getMotor(uint8_t ID);

	/**
	 * 	@brief
	 *
	 * 	@param tiltOffset
	 *
	 *	@details
	 **/
	void setTiltOffset(double tiltOffset);

	/**
	 * 	@brief Generate the path for each leg for the Hexapod to walk.
	 *
	 *	@param radialDistance The distance from the center of the Hexapod to the center of the support phase line.
	 *	@param direction The direction of the Hexapod .
	 *	@param strideLength The stride length of each leg.
	 *	@param stepHeight The step height of each leg.
	 *	@param bodyHeight The body height of the Hexapod.
	 *	@param slopePitch The pitch of the slope the Hexapod is walking on.
	 *	@param slopeRoll The roll of the slope the Hexapod is walking on.
	 *	@param bodyRotation An offset for the body twist angle.
	 *
	 *	@details
	 **/
	void makePath(double footVelocity,
			double bodyHeight,
			double crabAngle,
			double gaitPeriod,
			double radialDistance,
			double angleOffset,
			double stepHeight,
			double phaseOffset,
			double dutyFactor,
			double slopePitch,
			double slopeRoll);

	/**
	 * 	@brief Move the current goal position to the next point in the path.
	 *
	 *	@details
	 **/
	void moveNextPathPoint(void);

	/**
	 * 	@brief	Writes one or more registers of all motors in the leg.
	 *
	 *	@param reg The register to begin writing to.
	 *	@param data A pointer to the data to write.
	 *	@param dataLength The length of data to write.
	 *
	 *	@details
	 **/
	void writeRegs(enum CONTROL_TABLE reg, uint8_t * data, uint8_t dataLength);

	/**
	 * 	@brief	Sets the offset values of each motor in the leg.
	 *
	 *	@param off1 The offset of the first motor in radians.
	 *	@param off2 The offset of the second motor in radians.
	 *	@param off3 The offset of the third motor in radians.
	 *
	 *	@details
	 **/
	void setOffsets(double off1, double off2, double off3);

	/**
	 * 	@brief	Sets the position of the leg in cartesian coordinates.
	 *
	 *	@param x The x position of the foot tip.
	 *	@param y The y position of the foot tip.
	 *	@param z The z position of the foot tip.
	 *
	 *	@details
	 **/
	void setPosition(double x, double y, double z);

	/**
	 * 	@brief Sets the position of the leg in cartesian coordinates.
	 *
	 *	@param xyz A pointer to an array holding the goal x, y and z positions of the foot tip.
	 *
	 *	@details
	 **/
	void setPosition(double * xyz);

	/**
	 * 	@brief Sets the position of the leg from motor theta values.
	 *
	 *	@param theta1 The theta value of motor 1 in the leg.
	 *	@param theta2 The theta value of motor 2 in the leg.
	 *	@param theta3 The theta value of motor 3 in the leg.
	 *
	 *	@details
	 **/
	void setTheta(double theta1, double theta2, double theta3);

	/**
	 * 	@brief Sets the position of the leg from motor theta values.
	 *
	 * 	@param theta A pointer to an array holding the goal theta values of each of the motors in the leg.
	 *
	 *	@details
	 **/
	void setTheta(double * theta);

	/**
	 * 	@brief Sets the speed of each of the motors in the leg.
	 *
	 * 	@param xd The speed of leg in the x direction in m/s.
	 * 	@param yd The speed of leg in the y direction in m/s.
	 * 	@param zd The speed of leg in the z direction in m/s.
	 *
	 *	@details
	 **/
	void setSpeed(double xd, double yd, double zd);

	/**
	 * 	@brief Sets the speed of each of the motors in the leg.
	 *
	 * 	@param th1d The speed of motor 1 in the leg in RPM.
	 * 	@param th2d The speed of motor 2 in the leg in RPM.
	 * 	@param th3d The speed of motor 3 in the leg in RPM.
	 *
	 *	@details
	 **/
	void setSpeedRPM(double th1d, double th2d, double th3d);

	/**
	 * 	@brief Sets the speed of each of the motors in the leg.
	 *
	 *	@param theta_dot A pointer to an array holding the goal speed values of each of the motors in the leg in RPM.
	 *
	 *	@details
	 **/
	void setSpeed(double * theta_dot);

	/**
	 * 	@brief Sets the position bytes of each of the motors in the leg.
	 *
	 *	@details
	 **/
	void setPositionBits(void);

	/**
	 * 	@brief Sets the position bytes of each of the motors in the leg.
	 *
	 *	@param m1 The 2 byte position data from 0 to 1023 of motor 1 in the leg.
	 *	@param m2 The 2 byte position data from 0 to 1023 of motor 2 in the leg.
	 *	@param m3 The 2 byte position data from 0 to 1023 of motor 3 in the leg.
	 *
	 *	@details
	 **/
	void setPositionBits(uint16_t m1, uint16_t m2, uint16_t m3);

	/**
	 * 	@brief Sets the speed bytes of each of the motors in the leg.
	 *
	 *	@details
	 **/
	void setSpeedBits(void);

	/**
	 * 	@brief	Moves the leg to the current goal position.
	 *
	 *	@details
	 **/
	void move();
};

#endif
