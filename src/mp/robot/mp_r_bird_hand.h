#if !defined(MP_R_BIRD_HAND_H_)
#define MP_R_BIRD_HAND_H_

#include "mp/robot/mp_r_manip_and_conv.h"

namespace mrrocpp {
namespace mp {
namespace robot {
class bird_hand : public manip_and_conv
{

public:
	bird_hand(task::task &mp_object_l);
};
} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_BIRD_HAND_H_*/
