/*
 * Plan.h
 *
 *  Created on: Jun 24, 2010
 *      Author: ptrojane
 */

#ifndef PLAN_H_
#define PLAN_H_

#include <vector>

#include "HeadState.h"
#include "BaseState.h"
#include "PkmState.h"

//! Plan for a single agent
class Plan {
	//! list of head positions
	std::vector<HeadState> head;

	//! list of mobile base positions
	std::vector<BaseState> mbase;

	//! list of PKM positions
	std::vector<PkmState> pkm;
};

#endif /* PLAN_H_ */
