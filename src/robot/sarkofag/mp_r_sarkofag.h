#if !defined(MP_R_SARKOFAG_H_)
#define MP_R_SARKOFAG_H_

#include "base/mp/mp_r_motor_driven.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class sarkofag: public motor_driven {
public:
	sarkofag(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_SARKOFAG_H_*/

