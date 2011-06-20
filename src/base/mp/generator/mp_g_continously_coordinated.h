#ifndef MP_GEN_SPORADICALY_COORDINATED_H_
#define MP_GEN_SPORADICALY_COORDINATED_H_

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
class continously_coordinated : public generator
{

protected:
	/*!
	 * @brief stl map of all mp robots that have not reply
	 */
	common::robots_t robots_to_reply;


public:

	long cycle_counter;

	/**
	 * @brief Constructor
	 * @param _mp_task mp task object reference.
	 */
	continously_coordinated(task::task& _mp_task);

	bool next_step();

	virtual bool next_step_inside(void) = 0;

};

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif /*MP_GENERATORS_H_*/
