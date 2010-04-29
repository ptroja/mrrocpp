/*
 * ecp_g_get_position.h
 *
 *  Created on: Apr 29, 2010
 *      Author: rtulwin
 */

#ifndef _ECP_GEN_GET_POSITION_H_
#define _ECP_GEN_GET_POSITION_H_

#include "ecp/common/generator/ecp_g_delta.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class get_position : public delta {
	public:
		/**
		 * Constructor.
		 */
		get_position(common::task::task& _ecp_task, bool _is_synchronised, lib::POSE_SPECIFICATION pose_spec, int axes_num);
		/**
		 * Destructor.
		 */
		virtual ~get_position();
		/**
		 *
		 */
		virtual bool first_step();
		/**
		 *
		 */
		virtual bool next_step();
		/**
		 *
		 */
		double * get_position_array();
		/**
		 *	Dynamically allocated array filled with coordinates read from the robot.
		 */
		double * position;
		/**
		 * Type of the used coordinate system.
		 */\
		 lib::POSE_SPECIFICATION pose_spec;
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GEN_GET_POSITION_H_ */
