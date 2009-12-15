#if !defined(_ECP_T_SK_H)
#define _ECP_T_SK_H

#include "ecp/common/task/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class sk: public common::task::task  {
protected:
	generator::tff_nose_run* nrg;
	generator::y_edge_follow_force* yefg;
	generator::bias_edp_force* befg;
	bool save_activated;

public:
	// KONSTRUKTORY
	sk(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
