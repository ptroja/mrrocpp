#if !defined(_ECP_T_PROGPANEL_IRP6OT_H)
#define _ECP_T_PROGPANEL_IRP6OT_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {


class ecp_task_progpanel_irp6ot: public common::task::ecp_task  {
protected:
	common::generator::progpanel* ppg;
//	ecp_teach_in_generator* tig;

public:
	// KONSTRUKTORY
	ecp_task_progpanel_irp6ot(configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);

};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
