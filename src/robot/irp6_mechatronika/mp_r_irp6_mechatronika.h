#if !defined(MP_R_IRP6_MECHATRONIKA_H_)
#define MP_R_IRP6_MECHATRONIKA_H_

#include "base/mp/mp_robot.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class irp6_mechatronika : public robot
{
public:
	irp6_mechatronika(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_IRP6_MECHATRONIKA_H_*/
