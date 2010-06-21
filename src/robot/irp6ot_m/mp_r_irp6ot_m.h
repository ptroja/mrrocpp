#if !defined(MP_R_IRP6OT_M_H_)
#define MP_R_IRP6OT_M_H_

#include "mp/mp_r_motor_driven.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class irp6ot_m : public motor_driven
{
public:
	irp6ot_m(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_IRP6OT_M_H_*/
