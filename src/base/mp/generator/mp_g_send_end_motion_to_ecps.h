#ifndef MP_GEN_SEND_END_MOTION_TO_ECPS_H_
#define MP_GEN_SEND_END_MOTION_TO_ECPS_H_

/*!
 * @file
 * @brief File contains mp send_end_motion_to_ecps generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include "base/mp/generator/mp_generator.h"

namespace mrrocpp {
namespace mp {
namespace generator {

/*!
 * @brief Generator that sends END_MOTION command to coordinated robots
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup mp
 */
class send_end_motion_to_ecps : public generator
{
public:

	/**
	 * @brief Constructor
	 * @param _mp_task mp task object reference.
	 */
	send_end_motion_to_ecps(task::task& _mp_task);

	bool first_step();
	bool next_step();
};

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif /*MP_GENERATORS_H_*/
