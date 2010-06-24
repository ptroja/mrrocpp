/*
 * BaseState.h
 *
 *  Created on: Jun 24, 2010
 *      Author: ptrojane
 */

#ifndef BASESTATE_H_
#define BASESTATE_H_

#include "State.h"

class BaseState : State {
	double bx,by,bz;
	double theta;
	int ind;
};

#endif /* BASESTATE_H_ */
