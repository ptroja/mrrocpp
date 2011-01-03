#if !defined(_ECP_SUB_TASK_SMOOTH_GEN_TEST_H)
#define _ECP_SUB_TASK_SMOOTH_GEN_TEST_H

#include "base/ecp/ecp_sub_task.h"
#include "application/generator_tester/ecp_mp_st_smooth_gen_test.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
class newsmooth;
}

namespace sub_task {

class sub_task_smooth_gen_test : public sub_task
{

private:
	generator::newsmooth * sgenjoint;
	generator::newsmooth * sgenmotor;
	generator::newsmooth * sgeneuler;
	generator::newsmooth * sgenangle;

	bool track;
	bool postument;
	std::string network_path;

public:
	sub_task_smooth_gen_test(task::task & _ecp_t);
	~sub_task_smooth_gen_test();

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
