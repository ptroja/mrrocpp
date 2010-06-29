/*
 * PkmState.h
 *
 *  Created on: Jun 24, 2010
 *      Author: ptrojane
 */

#ifndef PKMSTATE_H_
#define PKMSTATE_H_

#include "State.h"

class PkmState : State {
	double l1, l2, l3;
	double phi;
	double psi1, psi2, psi3;
	double rho;
	int ind;
};

#endif /* PKMSTATE_H_ */
