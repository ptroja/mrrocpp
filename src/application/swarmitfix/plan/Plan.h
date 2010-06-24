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

class Plan {
	std::vector<HeadState> head;
	std::vector<BaseState> mbase;
	std::vector<PkmState> pkm;
};

#endif /* PLAN_H_ */
