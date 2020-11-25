/**
 * @file Path.cpp
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief Path class functions.
 **/

#include "Path.h"

Path::Path(void){

}

Path::~Path(void){

}

void Path::makePath(double dutyFactor, double gaitPeriod){	//XYZ
	double dt = gaitPeriod / pathSize;
	int supportSize = round(pathSize * dutyFactor); // not sure if round is supported
	int swingSize = pathSize - supportSize;

	/** SUPPORT PHASE GENERATION **/
	// Start point is the end point
	pathXYZ[0].p1 = startPoint.p1;
	pathXYZ[0].p2 = startPoint.p2;
	pathXYZ[0].p3 = startPoint.p3;

	double dx = (endPoint.p1 - startPoint.p1) / supportSize;
	double dy = (endPoint.p2 - startPoint.p2) / supportSize;
	double dz = (endPoint.p3 - startPoint.p3) / supportSize;

	pathXYZ_dot[0].p1 = dx / dt;
	pathXYZ_dot[0].p2 = dy / dt;
	pathXYZ_dot[0].p3 = dz / dt;

	// Support path loop
	for(int i=0; i < supportSize-1; i++){
		// new path = previous point + increment
		pathXYZ[i+1].p1 = pathXYZ[i].p1 + dx;
		pathXYZ[i+1].p2 = pathXYZ[i].p2 + dy;
		pathXYZ[i+1].p3 = pathXYZ[i].p3 + dz;
		// new velocity = previous velocity
		pathXYZ_dot[i+1].p1 = pathXYZ_dot[i].p1;
		pathXYZ_dot[i+1].p2 = pathXYZ_dot[i].p2;
		pathXYZ_dot[i+1].p3 = pathXYZ_dot[i].p3;
	}

	/** SWING PHASE GENERATION **/
	double tf = swingSize * dt;
	// X variables
	double xi = this->endPoint.p1;
	double xv = this->midPoint.p1;
	double xf = this->startPoint.p1;
	double ax0 = this->endPoint.p1;
	double ax1 = 0;
	double ax2 = 0;
	double ax3 = 	(2/	(tf*tf*tf))*			(32*(xv - xi) - 11*	(xf - xi));
	double ax4 = -	(3/	(tf*tf*tf*tf))*			(64*(xv - xi) - 27*	(xf - xi));
	double ax5 = 	(3/	(tf*tf*tf*tf*tf))*		(64*(xv - xi) - 30*	(xf - xi));
	double ax6 = -	(32/(tf*tf*tf*tf*tf*tf))*	(2*	(xv - xi) - 1*	(xf - xi));

	// Y variables
	double yi = this->endPoint.p2;
	double yv = this->midPoint.p2;
	double yf = this->startPoint.p2;
	double ay0 = this->endPoint.p2;
	double ay1 = 0;
	double ay2 = 0;
	double ay3 = 	(2/	(tf*tf*tf))*			(32*(yv - yi) - 11*	(yf - yi));
	double ay4 = -	(3/	(tf*tf*tf*tf))*			(64*(yv - yi) - 27*	(yf - yi));
	double ay5 = 	(3/	(tf*tf*tf*tf*tf))*		(64*(yv - yi) - 30*	(yf - yi));
	double ay6 = -	(32/(tf*tf*tf*tf*tf*tf))*	(2*	(yv - yi) - 1*	(yf - yi));

	// Z variables
	double zi = this->endPoint.p3;
	double zv = this->midPoint.p3;
	double zf = this->startPoint.p3;
	double az0 = this->endPoint.p3;
	double az1 = 0;
	double az2 = 0;
	double az3 = 	(2/	(tf*tf*tf))*			(32*(zv - zi) - 11*	(zf - zi));
	double az4 = -	(3/	(tf*tf*tf*tf))*			(64*(zv - zi) - 27*	(zf - zi));
	double az5 = 	(3/	(tf*tf*tf*tf*tf))*		(64*(zv - zi) - 30*	(zf - zi));
	double az6 = -	(32/(tf*tf*tf*tf*tf*tf))*	(2*	(zv - zi) - 1*	(zf - zi));

	double t = 0;

	// Swing path loop
	for(int i=supportSize; i < pathSize; t+=dt, i++){
		pathXYZ[i].p1 = ax6*t*t*t*t*t*t + ax5*t*t*t*t*t + ax4*t*t*t*t + ax3*t*t*t + ax2*t*t + ax1*t + ax0;
		pathXYZ[i].p2 = ay6*t*t*t*t*t*t + ay5*t*t*t*t*t + ay4*t*t*t*t + ay3*t*t*t + ay2*t*t + ay1*t + ay0;
		pathXYZ[i].p3 = az6*t*t*t*t*t*t + az5*t*t*t*t*t + az4*t*t*t*t + az3*t*t*t + az2*t*t + az1*t + az0;

		pathXYZ_dot[i].p1 = (6*ax6*t*t*t*t*t + 5*ax5*t*t*t*t + 4*ax4*t*t*t + 3*ax3*t*t + 2*ax2*t + ax1);
		pathXYZ_dot[i].p2 = (6*ay6*t*t*t*t*t + 5*ay5*t*t*t*t + 4*ay4*t*t*t + 3*ay3*t*t + 2*ay2*t + ay1);
		pathXYZ_dot[i].p3 = (6*az6*t*t*t*t*t + 5*az5*t*t*t*t + 4*az4*t*t*t + 3*az3*t*t + 2*az2*t + az1);
	}


}

// Starts the path generation process
void Path::setPoints(	double footVelocity,
						double bodyHeight,
						double crabAngle,
						double gaitPeriod,
						double legNumber,
						double radialDistance,
						double angleOffset,
						double stepHeight,
						double dutyFactor,
						double slopePitch,
						double slopeRoll){		//one leg at a time

	if(true){
//		direction -= M_PI;
		this->dutyFactor = dutyFactor;

		double strideLength = footVelocity * dutyFactor * gaitPeriod;
		double legAngle = -(M_PI/3.0)*(legNumber-1); // Angle between leg and body coordinate frames

//		double startDeltaH = (strideLength/2.0)*(tan(slopePitch)*cos(alpha) + tan(slopeRoll)*sin(alpha));
//		double endDeltaH = -(strideLength/2.0)*(tan(slopePitch)*cos(alpha) + tan(slopeRoll)*sin(alpha));

		double startDeltaH = 0;
		double endDeltaH = 0;

		this->midPoint.p1 = this->bodyRadius*cos(legAngle) + radialDistance*cos(legAngle + angleOffset);
		this->midPoint.p2 = this->bodyRadius*sin(legAngle) + radialDistance*sin(legAngle + angleOffset);
		this->midPoint.p3 = -bodyHeight + stepHeight + 14.0;

		this->startPoint.p1 = this->midPoint.p1 + (strideLength/2.0)*cos(crabAngle);
		this->startPoint.p2 = this->midPoint.p2 + (strideLength/2.0)*sin(crabAngle);
		this->startPoint.p3 = -bodyHeight + 14.0;

		this->endPoint.p1 = this->midPoint.p1 - (strideLength/2.0)*cos(crabAngle);
		this->endPoint.p2 = this->midPoint.p2 - (strideLength/2.0)*sin(crabAngle);
		this->endPoint.p3 = -bodyHeight + 14.0;

		this->makePath(dutyFactor, gaitPeriod);
	}



}
