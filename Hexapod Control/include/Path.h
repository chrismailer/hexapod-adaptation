/**
 * @file Path.h
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief Path class
 **/

#ifndef INCLUDE_PATH_H
#define INCLUDE_PATH_H

#include "stm32f4xx.h"
#include "Point.h"
#include "Math.h"

/**
 * 	@brief Path Class
 *
 *	@details
 *
 **/
class Path{

private:
	/** @brief A point object holding the starting points for the legs path. **/
	Point<double> startPoint;
	/** @brief A point object holding the middle points for the legs path. **/
	Point<double> midPoint;
	/** @brief A point object holding the end points for the legs path. **/
	Point<double> endPoint;

	/**
	 * 	@brief Generates the path points from the starting, middle and end points.
	 *
	 *	@details
	 *
	 **/
	void makePath(double dutyFactor, double gaitPeriod);

public:

	/** @brief The path size that is to be generated. **/
	#define pathSizeDef 120		//note also update currentPathPoint initial value in Leg.cpp, THIS MUST BE AN ODD NUMBER

	/** @brief Integer version of the path size. **/
	const int pathSize = pathSizeDef;

//	/** @brief Final time for path generation.  **/
//	double tf;

	/** @brief Duration of gait cycle  **/
	double gaitPeriod;

	/** @brief Percentage of time spent in support phase.  **/
	double dutyFactor;


	/** @brief An array holding the path points as XYZ values. **/
	Point<double> pathXYZ[pathSizeDef];

	/** @brief An array holding the path velocities as XYZ_dot values. **/
	Point<double> pathXYZ_dot[pathSizeDef];

	/** @brief The hexapod's body radius. **/
	const double bodyRadius = 125.54;

	/**
	 * @brief Default constructor for the class.
	 *
	 * @details Class constructor
	 *
	 **/
	Path(void);

	/**
	 * @brief Default destructor for the class.
	 *
	 * @details Class destructor cleans up memory by deleting pointers and lists.
	 *
	 **/
	~Path(void);

	/**
	 * 	@brief Sets the path parameters and generates the path points.
	 *	@param footVelocity The foot tip velocity.
	 *	@param bodyHeight The body height of the Hexapod.
	 *	@param crabAngle The walking direction of the Hexapod.
	 *	@param gaitPeriod The time to complete a leg gait cycle.
	 *	@param legNumber The leg number.
	 *	@param radialDistance The distance from the start of the leg to the center of the support phase line.
	 *	@param angleOffset The offset angle of a leg.
	 *	@param stepHeight The step height of each leg.
	 *	@param dutyFactor The percentage of the gait cycle spent in support
	 *	@param slopePitch The pitch of the slope the Hexapod is walking on.
	 *	@param slopeRoll The roll of the slope the Hexapod is walking on.
	 *
	 *	@details
	 *
	 **/
	void setPoints(double footVelocity, double bodyHeight, double crabAngle, double gaitPeriod, double legNumber, double radialDistance, double angleOffset, double stepHeight, double dutyFactor, double slopePitch, double slopeRoll);

};

#endif
