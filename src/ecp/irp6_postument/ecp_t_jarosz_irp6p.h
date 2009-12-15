#if !defined(_ECP_T_JAROSZ_IRP6P_H)
#define _ECP_T_JAROSZ_IRP6P_H

#include "ecp/common/task/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

class jarosz: public common::task::task  {

public:
	// KONSTRUKTORY
	jarosz(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
