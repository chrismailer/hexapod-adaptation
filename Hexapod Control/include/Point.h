/**
 * @file Point.h
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief Point class
 **/

#ifndef INCLUDE_POINT_H
#define INCLUDE_POINT_H

#include "enumDefinitions.h"

/**
 * 	@brief Point Template Class
 *
 *	@details A Point class containing three points which can be of any variable type.
 *	The type is specified as a template to the class.
 *
 **/
template <class T> class Point{
private:
	/** @brief An array to populate when converting the point to an array.  **/
	T pointArray[3] = {0,0,0};

public:
	/** @brief The first element of the point **/
	T p1;
	/** @brief The second element of the point  **/
	T p2;
	/** @brief The third element of the point  **/
	T p3;

	/**
	 * @brief Constructor for the class.
	 *
	 * @details Class constructor initializing all point values to 0.
	 *
	 **/
	Point<T>(void){
		this->p1 = 0;
		this->p2 = 0;
		this->p3 = 0;
	}

	/**
	 * @brief Constructor for the class.
	 *
	 * @details Class constructor initializing all point values to the values given.
	 *
	 * @param val1 The value to set the first element of the point to.
	 * @param val2 The value to set the second element of the point to.
	 * @param val3 The value to set the third element of the point to.
	 * @param type A purely indicative variable to help the user know whether the point is an XYZ value or THETA values.
	 *
	 **/
	Point(T val1, T val2, T val3, enum MOVEMENT type){
		this->p1 = val1;
		this->p2 = val2;
		this->p3 = val3;

		UNUSED(type);
	}

	/**
	 * @brief Constructor for the class.
	 *
	 * @details Class constructor initializing all point values to the values given.
	 *
	 * @param val A pointer to an array containing the values to set the elements of the point to.
	 * @param type A purely indicative variable to help the user know whether the point is an XYZ value or THETA values.
	 *
	 **/
	Point(T * val, enum MOVEMENT type){
		this->p1 = val[0];
		this->p2 = val[1];
		this->p3 = val[2];

		UNUSED(type);
	}

	/**
	 * @brief Default destructor for the class.
	 *
	 * @details Class destructor cleans up memory by deleting pointers and lists.
	 *
	 **/
	~Point(void){

	}

	/**
	 * @brief Converts the point to an array.
	 *
	 * @return A pointer to the array.
	 *
	 **/
	T * toArray(void){
		pointArray[0] = p1;
		pointArray[1] = p2;
		pointArray[2] = p3;
		return pointArray;
	}

};

#endif
