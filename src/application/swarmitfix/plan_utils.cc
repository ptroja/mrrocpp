/*
 * plan_utils.cc
 *
 *  Created on: Nov 10, 2011
 *      Author: ptroja
 */

#include "plan.hxx"

#include "base/lib/mrmath/ft_v_vector.h"
#include "base/lib/mrmath/homog_matrix.h"

using namespace mrrocpp::lib;

Homog_matrix PlanItem2Pose(const Pkm::ItemType & item)
{
	Xyz_Euler_Zyz_vector pose(
			item.l1(), item.l2(), item.l3(),
			item.psi1(), item.psi2(), item.psi3());

	return pose;
}

