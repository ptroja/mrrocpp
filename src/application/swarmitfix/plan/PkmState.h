/*
 * PkmState.h
 *
 *  Created on: Jun 24, 2010
 *      Author: ptrojane
 */

#ifndef PKMSTATE_H_
#define PKMSTATE_H_

#include "State.h"

//! State of the PKM as specified by in the plan
class PkmState : State {
	//! extensions of the first, second and third leg
	double l1, l2, l3;

	//! rotation angle of local PKM coordinate system, relative to the orientation of base
	double phi;

	//! rotation angles of the head support
	double psi1, psi2, psi3;

	//! additional rotation angle of the head
	double rho;

	//! TODO
	int ind;
};

#endif /* PKMSTATE_H_ */
