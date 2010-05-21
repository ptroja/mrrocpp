/*
 * ecp_g_constant_velocity.h
 *
 *  Created on: May 18, 2010
 *      Author: rtulwin
 */

#ifndef _ECP_G_CONSTANT_VELOCITY_H_
#define _ECP_G_CONSTANT_VELOCITY_H_

#include "ecp/common/generator/ecp_g_multiple_position.h"
#include "lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * Generator moves the robot with a constant velocity.
 */
class constant_velocity : public multiple_position {
	public:
		/**
		 * Constructor. Sets the axes_num and pose_spec variables.
		 * @param axes_num number of axes for a given robot and representation
		 * @param pose_spec representation in which the robot position is expressed
		 */
		constant_velocity(common::task::task& _ecp_task, bool _is_synchronised, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num);
		/**
		 * Destructor.
		 */
		virtual ~constant_velocity();
		/**
		 * Implementation of the first_step method.
		 */
		virtual bool first_step();
		/**
		 * Implementation of the next_step method.s
		 */
		virtual bool next_step();

	private:
		/**
		 * Number of axes for a given robot in used representation.
		 */
		int axes_num;
		/**
		 * Type of the used representation.
		 */
		 lib::ECP_POSE_SPECIFICATION pose_spec;
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_G_CONSTANT_VELOCITY_H_ */
