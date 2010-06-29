/*
 * ecp_g_get_position.h
 *
 *  Created on: Apr 29, 2010
 *      Author: rtulwin
 */

#ifndef _ECP_GEN_GET_POSITION_H_
#define _ECP_GEN_GET_POSITION_H_

#include "lib/mrmath/mrmath.h"
#include "base/ecp/ecp_generator.h"

#include <vector>

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * Generator used to get the actual position of the robot in the given representation and form.
 */
class get_position : public generator {
	public:
		/**
		 * Constructor. Creates a position vector. Sets the axes_num and pose_spec variables.
		 * @param axes_num number of axes for a given robot and representation
		 * @param pose_spec representation in which the robot position is expressed
		 */
		get_position(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num);
		/**
		 * Destructor.
		 */
		virtual ~get_position();
		/**
		 * Implementation of the first_step method.
		 */
		virtual bool first_step();
		/**
		 * Implementation of the next_step method. In case of this generator, this method is executed only once. Used to get the actual position of the robot.
		 */
		virtual bool next_step();
		/**
		 * Returns actual position.
		 * @return array containing actual robot position expressed in representation specified by pose_spec variable
		 */
		vector<double> get_position_vector();

	private:
		/**
		 * Vector filled with coordinates read from the robot.
		 */
		vector<double> position;
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

#endif /* _ECP_GEN_GET_POSITION_H_ */
