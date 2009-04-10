#if !defined(MP_R_CONVEYOR_H_)
#define MP_R_CONVEYOR_H_

#include "mp/mp_r_irp6s_and_conv.h"
namespace mrrocpp {
namespace mp {
namespace common {
class conveyor_robot: public irp6s_and_conv_robot {

 public:
  conveyor_robot (task::base &mp_object_l);
};
} // namespace common
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_CONVEYOR_H_*/
