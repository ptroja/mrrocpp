#if !defined(_ECP_T_JAROSZ_IRP6P_H)
#define _ECP_T_JAROSZ_IRP6P_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

class ecp_task_jarosz_irp6p: public common::task::base  {

public:
	// KONSTRUKTORY
	ecp_task_jarosz_irp6p(configurator &_config);
	~ecp_task_jarosz_irp6p();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
