#if !defined(MP_R_IRP6OT_M_H_)
#define MP_R_IRP6OT_M_H_

#include "base/mp/mp_r_motor_driven.h"
#include "lib/mrmath/ft_v_vector.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class irp6ot_m : public motor_driven
{
public:
	//! Buffer for direct force/torque readings
	DataBuffer<lib::Ft_vector> ft_data_buffer;

	irp6ot_m(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_IRP6OT_M_H_*/
