/*
 * BaseState.h
 *
 *  Created on: Jun 24, 2010
 *      Author: ptrojane
 */

#ifndef BASESTATE_H_
#define BASESTATE_H_

#include "State.h"

//! State of the mobile base as specified by in the plan
class BaseState : State {
	//! X,Y,Z-coordinates for the base’s reference point
	double bx,by,bz;

	//! rotation angle, which gives the orientation of base
	double theta;

	//! TODO
	int ind;
};

#endif /* BASESTATE_H_ */
