#if !defined(MP_R_IRP6_MECHATRONIKA_H_)
#define MP_R_IRP6_MECHATRONIKA_H_

#include "mp/mp_r_irp6s_and_conv.h"

class mp_irp6_mechatronika_robot: public mp_irp6s_and_conv_robot {

 public:
  mp_irp6_mechatronika_robot (mp_task &mp_object_l);
};

#endif /*MP_R_IRP6_MECHATRONIKA_H_*/
