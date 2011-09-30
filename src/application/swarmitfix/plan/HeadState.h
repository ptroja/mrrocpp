/*
 * HeadState.h
 *
 *  Created on: Jun 24, 2010
 *      Author: ptrojane
 */

#ifndef HEADSTATE_H_
#define HEADSTATE_H_

#include "State.h"

//! State of the head as specified by in the plan
class HeadState : State {
	//! rotation angles, specifying the orientation of head in 3-D space
	double alpha, beta, gamma;

	//! X,Y,Z-locations of the head reference point in world coordinates (relative to the bench)
	double hx, hy, hz;

	//! TODO
	int ind;
};

#endif /* HEADSTATE_H_ */
