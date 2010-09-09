#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "application/generator_tester/ecp_st_smooth_gen_test.h"
#include "generator/ecp/ecp_g_newsmooth.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

ecp_sub_task_smooth_gen_test::ecp_sub_task_smooth_gen_test(task & _ecp_t) :
	ecp_sub_task(_ecp_t) {

	sgenjoint = new generator::newsmooth(ecp_t, lib::ECP_JOINT, 6);
	sgenjoint->set_debug(true);

	sgenmotor = new generator::newsmooth(ecp_t, lib::ECP_MOTOR, 6);
	sgenmotor->set_debug(true);

	sgeneuler = new generator::newsmooth(ecp_t, lib::ECP_XYZ_EULER_ZYZ, 6);
	sgeneuler->set_debug(true);

	sgenangle = new generator::newsmooth(ecp_t, lib::ECP_XYZ_ANGLE_AXIS, 6);
	sgenangle->set_debug(true);
}

void ecp_sub_task_smooth_gen_test::conditional_execution() {

	std::vector<double> coordinates1(6);

	// POSTUMENT


	// JOINT ABSOLUTE
	ecp_t.sr_ecp_msg->message("Joint absolute");
	sgenjoint->reset();
	sgenjoint->set_absolute();
	coordinates1[0] = 0.101;
	coordinates1[1] = -1.942;
	coordinates1[2] = 0.149;
	coordinates1[3] = 1.082;
	coordinates1[4] = 3.658;
	coordinates1[5] = -2.738;
	sgenjoint->load_absolute_joint_trajectory_pose(coordinates1);
	coordinates1[0] = 0.001;
	coordinates1[1] = -1.742;
	coordinates1[2] = 0.049;
	coordinates1[3] = 1.382;
	coordinates1[4] = 3.658;
	coordinates1[5] = -2.338;
	sgenjoint->load_absolute_joint_trajectory_pose(coordinates1);
	coordinates1[0] = -0.101;
	coordinates1[1] = -1.542;
	coordinates1[2] = 0.049;
	coordinates1[3] = 1.182;
	coordinates1[4] = 3.658;
	coordinates1[5] = -2.738;
	sgenjoint->load_absolute_joint_trajectory_pose(coordinates1);

	if (sgenjoint->calculate_interpolate()) {
		sgenjoint->Move();
	}
	// JOINT ABSOLUTE END


	// JOINT RELATIVE
	ecp_t.sr_ecp_msg->message("Joint relative");
	sgenjoint->reset();
	sgenjoint->set_relative();
	coordinates1[0] = 0.1;
	coordinates1[1] = 0.0;
	coordinates1[2] = 0.0;
	coordinates1[3] = 0.0;
	coordinates1[4] = 0.0;
	coordinates1[5] = 0.0;
	sgenjoint->load_relative_joint_trajectory_pose(coordinates1);
	coordinates1[0] = 0.2;
	coordinates1[1] = 0.1;
	coordinates1[2] = -0.1;
	coordinates1[3] = 0.0;
	coordinates1[4] = 0.6;
	coordinates1[5] = 0.0;
	sgenjoint->load_relative_joint_trajectory_pose(coordinates1);
	coordinates1[0] = -0.3;
	coordinates1[1] = -0.1;
	coordinates1[2] = 0.1;
	coordinates1[3] = 0.0;
	coordinates1[4] = -0.6;
	coordinates1[5] = 0.0;
	sgenjoint->load_relative_joint_trajectory_pose(coordinates1);

	if (sgenjoint->calculate_interpolate()) {
		sgenjoint->Move();
	}
	// JOINT RELATIVE END


	// MOTOR ABSOLUTE
	ecp_t.sr_ecp_msg->message("Motor absolute");
	sgenmotor->reset();
	sgenmotor->set_absolute();
	coordinates1[0] = 0.0;
	coordinates1[1] = 20.0;
	coordinates1[2] = 5.0;
	coordinates1[3] = 0.0;
	coordinates1[4] = 20.0;
	coordinates1[5] = 0.0;
	sgenmotor->load_absolute_motor_trajectory_pose(coordinates1);
	coordinates1[0] = 0.0;
	coordinates1[1] = 40.0;
	coordinates1[2] = 0.0;
	coordinates1[3] = 50.0;
	coordinates1[4] = 6.0;
	coordinates1[5] = 0.0;
	sgenmotor->load_absolute_motor_trajectory_pose(coordinates1);
	coordinates1[0] = 0.0;
	coordinates1[1] = 0.0;
	coordinates1[2] = 0.0;
	coordinates1[3] = 0.0;
	coordinates1[4] = 0.0;
	coordinates1[5] = 0.0;
	sgenmotor->load_absolute_motor_trajectory_pose(coordinates1);

	if (sgenmotor->calculate_interpolate()) {
		sgenmotor->Move();
	}
	// MOTOR ABSOLUTE END


	// MOTOR RELATIVE
	ecp_t.sr_ecp_msg->message("Motor relative");
	sgenmotor->reset();
	sgenmotor->set_relative();
	coordinates1[0] = 0.0;
	coordinates1[1] = 20.0;
	coordinates1[2] = 0.0;
	coordinates1[3] = 0.0;
	coordinates1[4] = 20.0;
	coordinates1[5] = 0.0;
	sgenmotor->load_relative_motor_trajectory_pose(coordinates1);
	coordinates1[0] = 0.0;
	coordinates1[1] = -10.0;
	coordinates1[2] = 0.0;
	coordinates1[3] = 50.0;
	coordinates1[4] = -20.0;
	coordinates1[5] = 0.0;
	sgenmotor->load_relative_motor_trajectory_pose(coordinates1);
	coordinates1[0] = 0.0;
	coordinates1[1] = -10.0;
	coordinates1[2] = 0.0;
	coordinates1[3] = -50.0;
	coordinates1[4] = -0.0;
	coordinates1[5] = 0.0;
	sgenmotor->load_relative_motor_trajectory_pose(coordinates1);

	if (sgenmotor->calculate_interpolate()) {
		sgenmotor->Move();
	}
	// MOTOR RELATIVE END


	// EULER ABSOLUTE
	ecp_t.sr_ecp_msg->message("Euler absolute");
	sgeneuler->reset();
	sgeneuler->set_absolute();
	coordinates1[0] = 0.529991;
	coordinates1[1] = 1.706314;
	coordinates1[2] = 0.178314;
	coordinates1[3] = -2.185063;
	coordinates1[4] = 1.666544;
	coordinates1[5] = 2.328729;
	sgeneuler->load_absolute_euler_zyz_trajectory_pose(coordinates1);
	coordinates1[0] = 0.529991;
	coordinates1[1] = 1.7506314;
	coordinates1[2] = 0.178314;
	coordinates1[3] = -2.185063;
	coordinates1[4] = 1.566544;
	coordinates1[5] = 2.328729;
	sgeneuler->load_absolute_euler_zyz_trajectory_pose(coordinates1);
	coordinates1[0] = 0.529991;
	coordinates1[1] = 1.806314;
	coordinates1[2] = 0.178314;
	coordinates1[3] = -2.185063;
	coordinates1[4] = 1.766544;
	coordinates1[5] = 2.328729;
	sgeneuler->load_absolute_euler_zyz_trajectory_pose(coordinates1);

	if (sgeneuler->calculate_interpolate()) {
		sgeneuler->Move();
	}
	// EULER ABSOLUTE END


	// EULER RELATIVE
	ecp_t.sr_ecp_msg->message("Euler relative");
	sgeneuler->reset();
	sgeneuler->set_relative();
	coordinates1[0] = 0.0;
	coordinates1[1] = 0.1;
	coordinates1[2] = 0.0;
	coordinates1[3] = 0.15;
	coordinates1[4] = 0.0;
	coordinates1[5] = 0.0;
	sgeneuler->load_relative_euler_zyz_trajectory_pose(coordinates1);
	coordinates1[0] = 0.0;
	coordinates1[1] = -0.1;
	coordinates1[2] = 0.1;
	coordinates1[3] = -0.15;
	coordinates1[4] = 0.0;
	coordinates1[5] = 0.0;
	sgeneuler->load_relative_euler_zyz_trajectory_pose(coordinates1);
	coordinates1[0] = 0.0;
	coordinates1[1] = 0.0;
	coordinates1[2] = -0.1;
	coordinates1[3] = 0.0;
	coordinates1[4] = 0.0;
	coordinates1[5] = 0.0;
	sgeneuler->load_relative_euler_zyz_trajectory_pose(coordinates1);

	if (sgeneuler->calculate_interpolate()) {
		sgeneuler->Move();
	}
	// EULER RELATIVE END


	// ANGLE AXIS ABSOLUTE
	ecp_t.sr_ecp_msg->message("Angle axis absolute");
	sgenangle->reset();
	sgenangle->set_absolute();
	coordinates1[0] = 0.529987;
	coordinates1[1] = 1.806317;
	coordinates1[2] = 0.178306;
	coordinates1[3] = 1.307713;
	coordinates1[4] = -1.119889;
	coordinates1[5] = 0.104191;
	sgenangle->load_absolute_angle_axis_trajectory_pose(coordinates1);
	coordinates1[0] = 0.529987;
	coordinates1[1] = 1.706317;
	coordinates1[2] = 0.178306;
	coordinates1[3] = 1.207713;
	coordinates1[4] = -1.119889;
	coordinates1[5] = 0.004191;
	sgenangle->load_absolute_angle_axis_trajectory_pose(coordinates1);
	coordinates1[0] = 0.529987;
	coordinates1[1] = 1.806317;
	coordinates1[2] = 0.178306;
	coordinates1[3] = 1.367713;
	coordinates1[4] = -1.119889;
	coordinates1[5] = 0.104191;
	sgenangle->load_absolute_angle_axis_trajectory_pose(coordinates1);

	if (sgenangle->calculate_interpolate()) {
		sgenangle->Move();
	}
	// ANGLE AXIS ABSOLUTE END


	// ANGLE AXIS RELATIVE
	ecp_t.sr_ecp_msg->message("Angle axis relative");
	sgenangle->reset();
	sgenangle->set_relative();
	coordinates1[0] = 0.0;
	coordinates1[1] = 0.1;
	coordinates1[2] = 0.0;
	coordinates1[3] = 0.05;
	coordinates1[4] = 0.0;
	coordinates1[5] = 0.0;
	sgenangle->load_relative_angle_axis_trajectory_pose(coordinates1);
	coordinates1[0] = 0.0;
	coordinates1[1] = -0.1;
	coordinates1[2] = 0.1;
	coordinates1[3] = -0.05;
	coordinates1[4] = 0.0;
	coordinates1[5] = 0.0;
	sgenangle->load_relative_angle_axis_trajectory_pose(coordinates1);
	coordinates1[0] = 0.0;
	coordinates1[1] = 0.0;
	coordinates1[2] = -0.1;
	coordinates1[3] = 0.0;
	coordinates1[4] = 0.0;
	coordinates1[5] = 0.0;
	sgenangle->load_relative_angle_axis_trajectory_pose(coordinates1);

	if (sgenangle->calculate_interpolate()) {
		sgenangle->Move();
	}
	// ANGLE AXIS RELATIVE END


	// POSTUMENT END

}

ecp_sub_task_smooth_gen_test::~ecp_sub_task_smooth_gen_test() {
	delete sgenjoint;
	delete sgenmotor;
	delete sgeneuler;
	delete sgenangle;
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
