#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "application/generator_tester/ecp_st_const_vel_gen_test.h"
#include "generator/ecp/ecp_g_constant_velocity.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

ecp_sub_task_const_vel_gen_test::ecp_sub_task_const_vel_gen_test(task & _ecp_t) :
	ecp_sub_task(_ecp_t) {
	//yefg = new generator::y_edge_follow_force(_ecp_t, 8);
	cvgen = new generator::constant_velocity(ecp_t, lib::ECP_JOINT, 6);
}

void ecp_sub_task_const_vel_gen_test::conditional_execution() {

	//ecp_t.sr_ecp_msg->message("testttttttttttt");
	vector<double> coordinates1(6, 0.2);
	vector<double> coordinates2(6, 0.1);
	cvgen->load_absolute_joint_trajectory_pose(coordinates1);
	cvgen->load_absolute_joint_trajectory_pose(coordinates2);
	cvgen->calculate_interpolate();
	cvgen->Move();
}

} // namespace task

} // namespace common
} // namespace ecp
} // namespace mrrocpp
