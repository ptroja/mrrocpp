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

	cvgenjoint = new generator::constant_velocity(ecp_t, lib::ECP_JOINT, 6);
	cvgenjoint->set_debug(true);

	cvgenmotor = new generator::constant_velocity(ecp_t, lib::ECP_MOTOR, 6);
	cvgenmotor->set_debug(true);

	cvgeneuler = new generator::constant_velocity(ecp_t, lib::ECP_XYZ_EULER_ZYZ, 6);
	cvgeneuler->set_debug(true);

	//cvgenangle = new generator::constant_velocity(ecp_t, lib::ECP_XYZ_ANGLE_AXIS, 6);
	//cvgenangle->set_debug(true);
}

void ecp_sub_task_const_vel_gen_test::conditional_execution() {

	vector<double> coordinates1(6);

	// POSTUMENT

/*
	// JOINT ABSOLUTE
	cvgenjoint->reset();
	cvgenjoint->set_absolute();
	coordinates1[0] = 0.101;
	coordinates1[1] = -1.242;
	coordinates1[2] = 0.049;
	coordinates1[3] = 1.082;
	coordinates1[4] = 3.058;
	coordinates1[5] = -2.738;
	cvgenjoint->load_absolute_joint_trajectory_pose(coordinates1);
	coordinates1[0] = -0.101;
	coordinates1[1] = -1.542;
	coordinates1[2] = 0.049;
	coordinates1[3] = 1.182;
	coordinates1[4] = 3.658;
	coordinates1[5] = -2.738;
	cvgenjoint->load_absolute_joint_trajectory_pose(coordinates1);
	coordinates1[0] = -0.101;
	coordinates1[1] = -1.542;
	coordinates1[2] = 0.049;
	coordinates1[3] = 1.182;
	coordinates1[4] = 3.658;
	coordinates1[5] = -2.738;
	cvgenjoint->load_absolute_joint_trajectory_pose(coordinates1);

	if (cvgenjoint->calculate_interpolate()) {
		cvgenjoint->Move();
	}
	// JOINT ABSOLUTE END
*/
/*
	// JOINT RELATIVE
	cvgenjoint->reset();
	cvgenjoint->set_relative();
	coordinates1[0] = 0.1;
	coordinates1[1] = 0.0;
	coordinates1[2] = 0.0;
	coordinates1[3] = 0.0;
	coordinates1[4] = 0.0;
	coordinates1[5] = 0.0;
	cvgenjoint->load_relative_joint_trajectory_pose(coordinates1);
	coordinates1[0] = 0.2;
	coordinates1[1] = 0.0;
	coordinates1[2] = -0.1;
	coordinates1[3] = 0.0;
	coordinates1[4] = 0.6;
	coordinates1[5] = 0.0;
	cvgenjoint->load_relative_joint_trajectory_pose(coordinates1);
	coordinates1[0] = 0.0;
	coordinates1[1] = 0.2;
	coordinates1[2] = 0.0;
	coordinates1[3] = 0.2;
	coordinates1[4] = -0.3;
	coordinates1[5] = 0.0;
	cvgenjoint->load_relative_joint_trajectory_pose(coordinates1);

	if (cvgenjoint->calculate_interpolate()) {
		cvgenjoint->Move();
	}
	// JOINT RELATIVE END
*/
/*
	// MOTOR ABSOLUTE
	cvgenmotor->reset();
	cvgenmotor->set_absolute();
	coordinates1[0] = 0.0;
	coordinates1[1] = 20.0;
	coordinates1[2] = 0.0;
	coordinates1[3] = 0.0;
	coordinates1[4] = 20.0;
	coordinates1[5] = 0.0;
	cvgenmotor->load_absolute_motor_trajectory_pose(coordinates1);
	coordinates1[0] = 0.0;
	coordinates1[1] = 0.0;
	coordinates1[2] = 0.0;
	coordinates1[3] = 50.0;
	coordinates1[4] = 0.0;
	coordinates1[5] = 0.0;
	cvgenmotor->load_absolute_motor_trajectory_pose(coordinates1);
	coordinates1[0] = 0.0;
	coordinates1[1] = -10.0;
	coordinates1[2] = 0.0;
	coordinates1[3] = 0.0;
	coordinates1[4] = 15.0;
	coordinates1[5] = 0.0;
	cvgenmotor->load_absolute_motor_trajectory_pose(coordinates1);

	if (cvgenmotor->calculate_interpolate()) {
		cvgenmotor->Move();
	}
	// MOTOR ABSOLUTE END
*/
/*
	// MOTOR RELATIVE
	cvgenmotor->reset();
	cvgenmotor->set_relative();
	coordinates1[0] = 0.0;
	coordinates1[1] = 20.0;
	coordinates1[2] = 0.0;
	coordinates1[3] = 0.0;
	coordinates1[4] = 20.0;
	coordinates1[5] = 0.0;
	cvgenmotor->load_relative_motor_trajectory_pose(coordinates1);
	coordinates1[0] = 0.0;
	coordinates1[1] = -20.0;
	coordinates1[2] = 0.0;
	coordinates1[3] = 50.0;
	coordinates1[4] = -20.0;
	coordinates1[5] = 0.0;
	cvgenmotor->load_relative_motor_trajectory_pose(coordinates1);
	coordinates1[0] = 0.0;
	coordinates1[1] = -30.0;
	coordinates1[2] = 0.0;
	coordinates1[3] = 0.0;
	coordinates1[4] = -5.0;
	coordinates1[5] = 0.0;
	cvgenmotor->load_relative_motor_trajectory_pose(coordinates1);

	if (cvgenmotor->calculate_interpolate()) {
		cvgenmotor->Move();
	}
	// MOTOR RELATIVE END
*/

	// EULER ABSOLUTE
	cvgeneuler->reset();
	cvgeneuler->set_absolute();
	coordinates1[0] = 0.530;
	coordinates1[1] = 1.406;
	coordinates1[2] = 0.078;
	coordinates1[3] = -2.185;
	coordinates1[4] = 1.767;
	coordinates1[5] = 2.229;
	cvgeneuler->load_absolute_euler_zyz_trajectory_pose(coordinates1);
	coordinates1[0] = 0.530;
	coordinates1[1] = 1.806;
	coordinates1[2] = 0.178;
	coordinates1[3] = -1.885;
	coordinates1[4] = 1.767;
	coordinates1[5] = 2.329;
	cvgeneuler->load_absolute_euler_zyz_trajectory_pose(coordinates1);
	coordinates1[0] = 0.530;
	coordinates1[1] = 1.806;
	coordinates1[2] = 0.178;
	coordinates1[3] = -2.185;
	coordinates1[4] = 1.767;
	coordinates1[5] = 2.329;
	cvgeneuler->load_absolute_euler_zyz_trajectory_pose(coordinates1);

	if (cvgeneuler->calculate_interpolate()) {
		cvgeneuler->Move();
	}
	// EULER ABSOLUTE END


	// POSTUMENT END

}

ecp_sub_task_const_vel_gen_test::~ecp_sub_task_const_vel_gen_test() {
	delete cvgenjoint;
	delete cvgenmotor;
	delete cvgeneuler;
	//delete cvgenangle;
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
