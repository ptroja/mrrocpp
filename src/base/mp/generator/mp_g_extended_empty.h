#ifndef MP_GEN_EXTENDED_EMPTY_H_
#define MP_GEN_EXTENDED_EMPTY_H_

/*!
 * @file
 * @brief File contains mp extended_empty generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include "base/mp/generator/mp_generator.h"

// generator for setting the next ecps state

namespace mrrocpp {
namespace mp {
namespace generator {

/*!
 * @brief Generator waiting for ECP task termination message of any robot coordinated
 * it optionally can be finished by sending MP trigger from UI
 * the trajectory is generated in ECPs
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup mp
 */
class extended_empty : public generator
{

protected:

	/**
	 * @brief if flag is set mp_trigger finishes generator execution
	 */
	bool activate_trigger;

public:

	/**
	 * @brief Constructor
	 * @param _mp_task mp task object reference.
	 */
	extended_empty(task::task& _mp_task);

	/**
	 * @brief sets desired total time to pass
	 * @param l_activate_trigger set activate_trigger flag
	 */
	void configure(bool l_activate_trigger);

	bool first_step();
	bool next_step();
};

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif /*MP_GENERATORS_H_*/
