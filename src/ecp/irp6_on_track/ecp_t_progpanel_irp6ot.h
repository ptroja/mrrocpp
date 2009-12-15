#if !defined(_ECP_T_PROGPANEL_IRP6OT_H)
#define _ECP_T_PROGPANEL_IRP6OT_H

#include "ecp/common/task/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {


class progpanel: public common::task::task  {
protected:
	common::generator::progpanel* ppg;
//	ecp_teach_in_generator* tig;

public:
	// KONSTRUKTORY
	progpanel(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
