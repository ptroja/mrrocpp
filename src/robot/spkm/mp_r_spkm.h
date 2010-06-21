#if !defined(MP_R_SPKM_H_)
#define MP_R_SPKM_H_

#include "mp/mp_r_motor_driven.h"

namespace mrrocpp {
namespace mp {
namespace robot {
class spkm : public motor_driven
{
public:
	spkm(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_SPKM_H_*/
