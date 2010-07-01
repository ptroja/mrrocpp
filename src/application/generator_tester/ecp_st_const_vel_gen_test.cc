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
	cvgen->set_debug(true);
}

void ecp_sub_task_const_vel_gen_test::conditional_execution() {

	//ecp_t.sr_ecp_msg->message("testttttttttttt");
	cvgen->reset();
	vector<double> coordinates1(6);
	coordinates1[0] = 0.101;
	coordinates1[1] = -1.242;
	coordinates1[2] = 0.049;
	coordinates1[3] = 1.082;
	coordinates1[4] = 3.058;
	coordinates1[5] = -2.738;
	cvgen->load_absolute_joint_trajectory_pose(coordinates1);
	coordinates1[0] = -0.101;
	coordinates1[1] = -1.542;
	coordinates1[2] = 0.049;
	coordinates1[3] = 1.182;
	coordinates1[4] = 3.658;
	coordinates1[5] = -2.738;
	cvgen->load_absolute_joint_trajectory_pose(coordinates1);
	coordinates1[0] = -0.101;
	coordinates1[1] = -1.542;
	coordinates1[2] = 0.049;
	coordinates1[3] = 1.182;
	coordinates1[4] = 3.658;
	coordinates1[5] = -2.738;
	cvgen->load_absolute_joint_trajectory_pose(coordinates1);
	if (cvgen->calculate_interpolate()) {
		cvgen->Move();
	}
}

ecp_sub_task_const_vel_gen_test::~ecp_sub_task_const_vel_gen_test() {
	delete cvgen;
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
