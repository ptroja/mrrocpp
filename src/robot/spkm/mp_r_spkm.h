#if !defined(MP_R_SPKM_H_)
#define MP_R_SPKM_H_

/*!
 * @file
 * @brief File contains mp robot class declaration for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "base/mp/mp_robot.h"
#include "robot/spkm/const_spkm.h"

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
public:
	/**
	 * @brief constructor
	 * @param mp_object_l mp task object reference
	 */
	spkm(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_SPKM_H_*/
