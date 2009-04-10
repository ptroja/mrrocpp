#if !defined(MP_R_CONVEYOR_H_)
#define MP_R_CONVEYOR_H_

#include "mp/mp_r_irp6s_and_conv.h"
namespace mrrocpp {
namespace mp {
namespace common {
class mp_conveyor_robot: public mp_irp6s_and_conv_robot {

 public:
  mp_conveyor_robot (task::mp_task &mp_object_l);
};
} // namespace common
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_CONVEYOR_H_*/
