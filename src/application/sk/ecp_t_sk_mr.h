#if !defined(_ECP_T_SPKM_SK_MR_TEST_H)
#define _ECP_T_SPKM_SK_MR_TEST_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_transparent.h"

namespace mrrocpp {
namespace ecp {
namespace sk_mr {
namespace task {

class sk_mr_test: public common::task::task {
protected:
	//generatory
	common::generator::transparent* gt;
	common::generator::sleep* g_sleep;
	common::generator::sk_mr* g_sk_mr;

public:
	// KONSTRUKTORY
	sk_mr_test(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

}
} // namespace sk_mr
} // namespace ecp
} // namespace mrrocpp

#endif
