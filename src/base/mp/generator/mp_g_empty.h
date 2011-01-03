#ifndef MP_GEN_EMPTY_H_
#define MP_GEN_EMPTY_H_

/*!
 * @file
 * @brief File contains mp empty generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include "base/mp/generator/mp_generator.h"

namespace mrrocpp {
namespace mp {
namespace generator {

/*!
 * @brief Generator waiting for ECP task termination message of any robot coordinated.
 * the trajectory is generated in ECPs
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup mp
 */
class empty : public generator
{

public:

	/**
	 * @brief Constructor
	 * @param _mp_task mp task object reference.
	 */
	empty(task::task& _mp_task);

	bool first_step();

	bool next_step();

};

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif /*MP_GENERATORS_H_*/
