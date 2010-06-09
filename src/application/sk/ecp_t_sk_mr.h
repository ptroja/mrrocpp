#if !defined(_ECP_T_SK_MR_H)
#define _ECP_T_SK_MR_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_force.h"
#include "application/sk/ecp_g_sk.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class sk_mr: public common::task::task {
protected:
	//generatory
	generator::tff_nose_run* nrg;
	generator::y_edge_follow_force* yefg;
	generator::bias_edp_force* befg;
	bool save_activated;

public:
	// KONSTRUKTORY
	sk_mr(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
