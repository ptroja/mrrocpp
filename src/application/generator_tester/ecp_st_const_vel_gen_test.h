#if !defined(_ECP_SUBTASK_CONST_VEL_GEN_TEST_H)
#define _ECP_SUBTASK_CONST_VEL_GEN_TEST_H

#include "base/ecp/ecp_subtask.h"
#include "application/generator_tester/ecp_mp_st_const_vel_gen_test.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
class constant_velocity;
}

namespace subtask {

class subtask_const_vel_gen_test : public subtask
{

private:
	generator::constant_velocity* cvgenjoint;
	generator::constant_velocity* cvgenmotor;
	generator::constant_velocity* cvgeneuler;
	generator::constant_velocity* cvgenangle;

	bool track;
	bool postument;
	bool conv;
        std::string network_path;

public:
	subtask_const_vel_gen_test(task::task & _ecp_t);
	~subtask_const_vel_gen_test();

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
