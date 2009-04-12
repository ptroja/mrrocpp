#if !defined(_ECP_T_TS_IRP6OT_H)
#define _ECP_T_TS_IRP6OT_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

class ecp_task_ts_irp6ot: public common::task::ecp_task  {

public:
	// KONSTRUKTORY
	ecp_task_ts_irp6ot(configurator &_config);
	~ecp_task_ts_irp6ot();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
