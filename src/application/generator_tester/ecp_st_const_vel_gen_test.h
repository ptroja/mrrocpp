#if !defined(_ECP_SUB_TASK_EDGE_FOLLOW_H)
#define _ECP_SUB_TASK_EDGE_FOLLOW_H

#include "base/ecp/ecp_task.h"
#include "application/generator_tester/ecp_mp_st_const_vel_gen_test.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
	class constant_velocity;
}

namespace task {

class ecp_sub_task_const_vel_gen_test : public ecp_sub_task
{

private:
	generator::constant_velocity* cvgen;

public:
	ecp_sub_task_const_vel_gen_test(task & _ecp_t);

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
