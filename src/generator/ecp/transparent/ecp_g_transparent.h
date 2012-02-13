#if !defined(_ECP_GENERATOR_T_H)
#define  _ECP_GENERATOR_T_H

/*!
 * @file
 * @brief File contains ecp transparent generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * it used in continuous coordination
 * the kinematic checking handler can be enabled and disabled
 *
 * @ingroup generators
 */

#include "ecp_mp_g_transparent.h"
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
 * @ingroup generators
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

	/**
	 * @brief generates first step of transition function
	 * @return terminal condition value
	 */
	bool first_step();

	/**
	 * @brief generates next steps (starting from the second) of transition function
	 * @return terminal condition value
	 */
	bool next_step();

	/**
	 * @brief communicates with EDP
	 * reimplemented here to handle kinematic exceptions
	 */
	void execute_motion(void);

	/**
	 * @brief method executed by dispatcher
	 */
	void conditional_execution();

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GENERATOR_T_H */
