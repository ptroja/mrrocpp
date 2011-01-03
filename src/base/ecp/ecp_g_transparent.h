#if !defined(_ECP_GENERATOR_T_H)
#define  _ECP_GENERATOR_T_H

/*!
 * @file
 * @brief File contains ecp transparent generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/*!
 * @brief transparent generator
 * used in continuous coordination
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class transparent : public common::generator::generator
{

public:
	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	transparent(common::task::task& _ecp_task);

	/**
	 * @brief bool flag deciding if Kinematic exceptions should finish task execution
	 */
	bool throw_kinematics_exceptions;

	bool first_step();

	bool next_step();

	/**
	 * @brief communicates with EDP
	 * reimplemented here to handle kinematic exceptions
	 */
	void execute_motion(void);
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GENERATOR_T_H */
