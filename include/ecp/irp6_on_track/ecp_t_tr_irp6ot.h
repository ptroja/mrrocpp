#if !defined(_ECP_T_TR_IRP6OT_H)
#define _ECP_T_TR_IRP6OT_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class tr: public common::task::base  {

public:
	// KONSTRUKTORY
	tr(configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);

	void catch_signal(int sig);

};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
