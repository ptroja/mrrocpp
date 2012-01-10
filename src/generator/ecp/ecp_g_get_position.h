/**
 * @file
 * @brief Contains declarations of the methods of get_position class.
 * @author rtulwin
 * @ingroup generators
 */

#ifndef _ECP_GEN_GET_POSITION_H_
#define _ECP_GEN_GET_POSITION_H_

#include <vector>

#include "base/lib/mrmath/mrmath.h"
#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * @brief Generator used to get the actual position of the robot in the given representation and form.
 *
 * @author rtulwin
 * @ingroup generators
 */
class get_position : public generator {
	public:
		/**
		 * Constructor. Creates a position vector. Sets the axes_num and pose_spec variables.
		 * @param _ecp_task current ecp task
		 * @param axes_num number of axes for a given robot and representation
		 * @param pose_spec representation in which the robot position is expressed
		 */
		get_position(task_t & _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num);
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
		const std::vector<double> & get_position_vector() const;

	private:
		/**
		 * Vector filled with coordinates read from the robot.
		 */
		std::vector<double> position;
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
