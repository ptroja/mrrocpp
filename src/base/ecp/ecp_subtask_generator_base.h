#if !defined(_ECP_SUBTASK_GENERATOR_BASE_H)
#define _ECP_SUBTASK_GENERATOR_BASE_H

/*!
 * @file
 * @brief File contains ecp subtask generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include "base/lib/com_buf.h"

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

const std::string EMPTY_SUBTASK_GENERATOR_NAME = "EMPTY_SUBTASK_GENERATOR_NAME";

class subtask_generator_base
{

public:

	/**
	 * @brief Unique class name
	 */
	lib::ecp_subtask_generator_name_t subtask_generator_name; // by Y - nazwa robota (track, postument etc.)

	subtask_generator_base() :
			subtask_generator_name(EMPTY_SUBTASK_GENERATOR_NAME)
	{

	}

	virtual ~subtask_generator_base()
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
