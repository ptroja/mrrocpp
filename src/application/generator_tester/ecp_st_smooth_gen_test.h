#if !defined(_ECP_SUBTASK_SMOOTH_GEN_TEST_H)
#define _ECP_SUBTASK_SMOOTH_GEN_TEST_H

#include "base/ecp/ecp_subtask.h"
#include "application/generator_tester/ecp_mp_st_smooth_gen_test.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
class newsmooth;
}

namespace subtask {

class subtask_smooth_gen_test : public subtask
{

private:
	generator::newsmooth * sgenjoint;
	generator::newsmooth * sgenmotor;
	generator::newsmooth * sgeneuler;
	generator::newsmooth * sgenangle;

	bool track;
	bool postument;
	bool conv;
	std::string network_path;

public:
	subtask_smooth_gen_test(task::task & _ecp_t);
	~subtask_smooth_gen_test();

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
