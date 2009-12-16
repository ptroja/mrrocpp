#if !defined(_ECP_T_PR_IRP6OT_H)
#define _ECP_T_PR_IRP6OT_H

#include "ecp/common/task/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class pr: public common::task::task  {
protected:
	common::generator::y_drawing_teach_in_force *tig;
	int ecp_tryb;
//	void *dtig;
	common::generator::y_nose_run_force* ynrlg;

	void short_move_up ();

public:
	// KONSTRUKTORY
	pr(lib::configurator &_config);
	~pr();

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);

};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
