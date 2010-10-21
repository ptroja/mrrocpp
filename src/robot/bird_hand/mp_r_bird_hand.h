#if !defined(MP_R_BIRD_HAND_H_)
#define MP_R_BIRD_HAND_H_

/*!
 * @file
 * @brief File contains mp robot class declaration for Bird Hand three finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup bird_hand
 */

#include "base/mp/mp_robot.h"
#include "const_bird_hand.h"

namespace mrrocpp {
namespace mp {
namespace robot {

/*!
 * @brief Bird Hand gripper mp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup bird_hand
 */
class bird_hand : public robot
{

public:
	/**
	 * @brief constructor
	 * @param mp_object_l mp task object reference
	 */
	bird_hand(task::task &mp_object_l);
};
} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_BIRD_HAND_H_*/
