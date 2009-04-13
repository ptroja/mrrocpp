#if !defined(_ECP_T_PR_IRP6OT_H)
#define _ECP_T_PR_IRP6OT_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class ecp_task_pr_irp6ot: public common::task::ecp_task  {
protected:
	common::generator::y_drawing_teach_in_force *tig;
	int ecp_tryb;
//	void *dtig;
	common::generator::y_nose_run_force* ynrlg;

	void short_move_up ();

public:
	// KONSTRUKTORY
	ecp_task_pr_irp6ot(configurator &_config);
	~ecp_task_pr_irp6ot();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
