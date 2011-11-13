#if !defined(MP_R_SPKM_H_)
#define MP_R_SPKM_H_

/*!
 * @file mp_r_spkm.h
 * @brief File contains mp robot class declaration for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "base/mp/mp_robot.h"

#include "base/lib/swarmtypes.h"
#include "dp_spkm.h"

namespace mrrocpp {
namespace mp {
namespace robot {

/*!
 * @brief SwarmItFix parallel manipulator mp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup spkm
 */
class spkm : public robot
{
	//! Buffer for notifications from ECP
	InputBuffer<lib::notification_t> notifyBuffer;

	//! Buffer for commands to ECP
	OutputBuffer<lib::spkm::next_state_t> nextstateBuffer;

public:
	/**
	 * @brief constructor
	 * @param mp_object_l mp task object reference
	 */
	spkm(const lib::robot_name_t & l_robot_name, task::task &mp_object_l);

	//! Access to the most recent notification message
	const int & notification;
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp

#endif /*MP_R_SPKM_H_*/
