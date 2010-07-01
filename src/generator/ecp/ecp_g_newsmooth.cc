/**
 * \file ecp_g_newsmooth.cc
 * \brief newsmooth class and its methods
 *
 * Contains bodies of the methods of newsmooth class.
 */

#include "generator/ecp/ecp_g_newsmooth.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

newsmooth::newsmooth(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num) :
				multiple_position<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose,
				ecp::common::generator::trajectory_interpolator::bang_bang_interpolator,
				ecp::common::generator::velocity_profile_calculator::bang_bang_profile> (_ecp_task) {

}

newsmooth::~newsmooth() {

}

void newsmooth::set_axes_num(int axes_num) {

}

bool newsmooth::first_step() {

	return true;
}

bool newsmooth::next_step()
{

	return true;
}

bool newsmooth::calculate_interpolate() {

	return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
