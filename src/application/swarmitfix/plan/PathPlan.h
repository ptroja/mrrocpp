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

class PathPlan {
	std::vector<Plan> agents;
	std::vector<double> D, H;
	std::vector<double> timePoints;
};

#endif /* PATHPLAN_H_ */
