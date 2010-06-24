#if !defined(MP_R_IRP6OT_TFG_H_)
#define MP_R_IRP6OT_TFG_H_

#include "base/mp/mp_r_motor_driven.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class irp6ot_tfg: public motor_driven {
public:
	irp6ot_tfg(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_IRP6OT_TFG_H_*/

