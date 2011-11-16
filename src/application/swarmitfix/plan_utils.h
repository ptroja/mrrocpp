/*
 * plan_utils.h
 *
 *  Created on: Nov 10, 2011
 *      Author: ptroja
 */

#ifndef PLAN_UTILS_H_
#define PLAN_UTILS_H_

#include "plan.hxx"
#include "base/lib/mrmath/homog_matrix.h"

namespace mrrocpp {
namespace robot {

mrrocpp::lib::Homog_matrix PlanItem2Pose(const Pkm::ItemType & item);

}
}

#endif /* PLAN_UTILS_H_ */
