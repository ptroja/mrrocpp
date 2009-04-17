#if !defined(_ECP_T_VISLX_IRP6OT_H)
#define _ECP_T_VISLX_IRP6OT_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class vislx: public common::task::task  {

public:
	// KONSTRUKTORY
	vislx(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
