/*
 * HeadState.h
 *
 *  Created on: Jun 24, 2010
 *      Author: ptrojane
 */

#ifndef HEADSTATE_H_
#define HEADSTATE_H_

#include "State.h"

class HeadState : State {
	double alpha, beta, gamma;
	double hx, hy, hz;
	int ind;
};

#endif /* HEADSTATE_H_ */
