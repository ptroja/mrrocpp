#ifndef __SG_IRP6OT_M_H
#define __SG_IRP6OT_M_H

#include "base/edp/edp_typedefs.h"
#include "base/edp/servo_gr.h"

namespace mrrocpp {
namespace edp {
namespace irp6ot_m {

// Forward declaration.
class effector;

class servo_buffer: public common::servo_buffer
{
public:
	effector &master;

	void load_hardware_interface(void);

	servo_buffer(effector &_master);
};

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp

#endif
