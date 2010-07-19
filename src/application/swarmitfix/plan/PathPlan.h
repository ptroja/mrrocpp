/*
 * PathPlan.h
 *
 *  Created on: Jun 24, 2010
 *      Author: ptrojane
 */

#ifndef PATHPLAN_H_
#define PATHPLAN_H_

#include <vector>

#include "Plan.h"

//! Plan for multiple-agent system
class PathPlan {
	//! Plans for individual agents
	std::vector<Plan> agents;

	//! TODO:
	std::vector<double> D, H;

	//! list of the time-point values for individual indices
	std::vector<double> timePoints;
};

#endif /* PATHPLAN_H_ */
