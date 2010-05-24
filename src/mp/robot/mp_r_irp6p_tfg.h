#if !defined(MP_R_IRP6P_TFG_H_)
#define MP_R_IRP6P_TFG_H_

#include "mp/robot/mp_r_manip_and_conv.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class irp6p_tfg : public manip_and_conv
{
public:
	irp6p_tfg(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_IRP6P_TFG_H_*/
