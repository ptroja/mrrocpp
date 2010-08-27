#if !defined(MP_R_BIRD_HAND_H_)
#define MP_R_BIRD_HAND_H_

/*!
 * @file mp_r_bird_hand.h
 * @brief File contains mp robot class declaration for Bird Hand three finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup bird_hand
 */


#include "base/mp/mp_r_motor_driven.h"

namespace mrrocpp {
namespace mp {
namespace robot {
class bird_hand : public motor_driven
{

public:
	bird_hand(task::task &mp_object_l);
};
} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_BIRD_HAND_H_*/
