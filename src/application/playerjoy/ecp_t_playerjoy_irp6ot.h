#if !defined(_ECP_T_PLAYERJOY_IRP6OT_H)
#define _ECP_T_PLAYERJOY_IRP6OT_H

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

class playerjoy: public common::task::task  {
protected:
	common::generator::playerjoy* pjg;

public:
	// KONSTRUKTORY
	playerjoy(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
