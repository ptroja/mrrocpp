#if !defined(MP_R_IRP6_POSTUMENT_H_)
#define MP_R_IRP6_POSTUMENT_H_

#include "mp/mp_r_irp6s_and_conv.h"
namespace mrrocpp {
namespace mp {
namespace robot {
class irp6_postument: public irp6s_and_conv {

 public:
  irp6_postument (task::task &mp_object_l);
};
} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_IRP6_POSTUMENT_H_*/
