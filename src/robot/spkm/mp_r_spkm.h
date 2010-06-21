#if !defined(MP_R_SPKM_H_)
#define MP_R_SPKM_H_

#include "mp/robot/mp_r_manip_and_conv.h"

namespace mrrocpp {
namespace mp {
namespace robot {
class spkm : public manip_and_conv
{
public:
	spkm(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_SPKM_H_*/
