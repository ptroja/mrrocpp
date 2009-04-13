#if !defined(_ECP_T_TW_H)
#define _ECP_T_TW_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class tw: public common::task::base  {
protected:
	generator::tff_nose_run* nrg;
	generator::y_edge_follow_force* yefg;
	generator::bias_edp_force* befg;
	bool save_activated;

public:
	// KONSTRUKTORY
	tw(configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
