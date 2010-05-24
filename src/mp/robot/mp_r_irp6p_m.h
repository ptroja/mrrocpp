#if !defined(MP_R_IRP6P_M_H_)
#define MP_R_IRP6P_M_H_

#include "mp/robot/mp_r_manip_and_conv.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class irp6p_m: public manip_and_conv {
public:
	irp6p_m(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_IRP6P_M_H_*/
