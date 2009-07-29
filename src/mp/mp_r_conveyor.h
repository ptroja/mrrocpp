#if !defined(MP_R_CONVEYOR_H_)
#define MP_R_CONVEYOR_H_

#include "mp/mp_r_irp6s_and_conv.h"
namespace mrrocpp {
namespace mp {
namespace robot {
class conveyor: public irp6s_and_conv {

 public:
  conveyor (task::task &mp_object_l);
};
} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_CONVEYOR_H_*/
