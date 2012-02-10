/**
 * @file
 * @brief Contains declarations of the methods of constant_velocity class.
 * @author rtulwin
 * @ingroup generators
 */

#ifndef _ECP_G_CONSTANT_VELOCITY_TFG_H_
#define _ECP_G_CONSTANT_VELOCITY_TFG_H_

#include "../../generator/ecp/constant_velocity/ecp_g_constant_velocity.h"

namespace mrrocpp {
namespace ecp {
namespace irp6_tfg {
namespace generator {

class constant_velocity : public common::generator::constant_velocity
{
public:
	/**
	 * Constructor. Sets the axes_num and pose_spec variables.
	 * @param _ecp_task current ecp task
	 * @param axes_num number of axes for a given robot and representation
	 * @param pose_spec representation in which the robot position is expressed
	 */
	constant_velocity(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num);

	void conditional_execution();

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_G_CONSTANT_VELOCITY_H_ */
