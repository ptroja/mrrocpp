#ifndef MP_GEN_SEND_END_MOTION_TO_ECPS_H_
#define MP_GEN_SEND_END_MOTION_TO_ECPS_H_

/*!
 * @file
 * @brief File contains mp common generators declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include "base/mp/generator/mp_generator.h"

// generator for setting the next ecps state

namespace mrrocpp {
namespace mp {
namespace generator {

// generator for sending end_motion message to ecps

class send_end_motion_to_ecps : public generator
{
public:

	// konstruktor
	send_end_motion_to_ecps(task::task& _mp_task);

	bool first_step();
	bool next_step();
};

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif /*MP_GENERATORS_H_*/
