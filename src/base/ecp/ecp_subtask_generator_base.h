#if !defined(_ECP_SUBTASK_GENERATOR_BASE_H)
#define _ECP_SUBTASK_GENERATOR_BASE_H

/*!
 * @file
 * @brief File contains ecp base generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include <boost/shared_ptr.hpp>

#include "base/ecp_mp/ecp_mp_generator.h"
#include "base/ecp/ecp_robot.h"
#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {

/*!
 * @brief Base class of all ecp generators
 * The generator both generates command and checks terminal condition
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class subtask_generator_base
{

public:

	subtask_generator_base()
	{
	}

	/**
	 * @brief executed by disptecher
	 */
	virtual void conditional_execution() = 0;
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GENERATOR_H */
