#if !defined(MP_R_BIRD_HAND_H_)
#define MP_R_BIRD_HAND_H_

#include "mp/mp_r_motor_driven.h"

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
