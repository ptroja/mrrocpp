#if !defined(MP_R_IRP6_ON_TRACK_H)
#define MP_R_IRP6_ON_TRACK_H

#include "mp/mp_r_irp6s_and_conv.h"
namespace mrrocpp {
namespace mp {
namespace robot {
class irp6_on_track_robot: public irp6s_and_conv_robot {

 public:
  irp6_on_track_robot (task::task &mp_object_l);
};
} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_IRP6_ON_TRACK_H*/
