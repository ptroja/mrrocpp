#if !defined(MP_R_IRP6_MECHATRONIKA_H_)
#define MP_R_IRP6_MECHATRONIKA_H_

#include "base/mp/mp_robot.h"
#include "robot/irp6m/const_irp6m.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class irp6m : public robot
{
public:
	irp6m(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_IRP6_MECHATRONIKA_H_*/
