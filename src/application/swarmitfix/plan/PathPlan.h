/*
 * PathPlan.h
 *
 *  Created on: Jun 24, 2010
 *      Author: ptrojane
 */

#ifndef PATHPLAN_H_
#define PATHPLAN_H_

#include <vector>
#include <string>

#include "Plan.h"

class PathPlan {
	std::vector<Plan> agents;
	std::vector<double> D, H;
	std::vector<double> timePoints;

public:
	void load(const std::string & filename);
};

#endif /* PATHPLAN_H_ */
