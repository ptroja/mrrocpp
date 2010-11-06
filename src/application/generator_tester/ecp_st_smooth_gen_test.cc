#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "application/generator_tester/ecp_st_smooth_gen_test.h"
#include "generator/ecp/ecp_g_newsmooth.h"

#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace sub_task {

sub_task_smooth_gen_test::sub_task_smooth_gen_test(task::task & _ecp_t) :
	sub_task(_ecp_t)
{

	if (_ecp_t.ecp_m_robot->robot_name == lib::irp6p_m::ROBOT_NAME) {
		sgenjoint = new generator::newsmooth(ecp_t, lib::ECP_JOINT, 6);
		sgenjoint->set_debug(true);

		sgenmotor = new generator::newsmooth(ecp_t, lib::ECP_MOTOR, 6);
		sgenmotor->set_debug(true);

		track = false;
		postument = true;

	} else if (_ecp_t.ecp_m_robot->robot_name == lib::irp6ot_m::ROBOT_NAME) {
		sgenjoint = new generator::newsmooth(ecp_t, lib::ECP_JOINT, 7);
		sgenjoint->set_debug(true);

		sgenmotor = new generator::newsmooth(ecp_t, lib::ECP_MOTOR, 7);
		sgenmotor->set_debug(true);

		track = true;
		postument = false;
	}

	sgeneuler = new generator::newsmooth(ecp_t, lib::ECP_XYZ_EULER_ZYZ, 6);
	sgeneuler->set_debug(true);

	sgenangle = new generator::newsmooth(ecp_t, lib::ECP_XYZ_ANGLE_AXIS, 6);
	sgenangle->set_debug(true);

	network_path = std::string(ecp_t.mrrocpp_network_path);
}

void sub_task_smooth_gen_test::conditional_execution()
{

	std::vector <double> coordinates1(6);
	std::vector <double> coordinates2(7);

	// JOINT ABSOLUTE
	sr_ecp_msg.message("Joint absolute");
	sgenjoint->reset();
	sgenjoint->set_absolute();
	if (track) {
		network_path += "src/application/generator_tester/trajectory.trj";
		sgenjoint->load_trajectory_from_file(network_path.c_str());

		coordinates2[0] = 0.1;
		coordinates2[1] = -0.067;
		coordinates2[2] = -1.342;
		coordinates2[3] = -0.064;
		coordinates2[4] = 1.109;
		coordinates2[5] = 1.573;
		coordinates2[6] = -2.664;
		sgenjoint->load_absolute_joint_trajectory_pose(coordinates2);
	} else if (postument) {
		coordinates1[0] = -0.104;
		coordinates1[1] = -1.542;
		coordinates1[2] = 0.020;
		coordinates1[3] = 1.404;
		coordinates1[4] = 3.358;
		coordinates1[5] = -2.538;
		sgenjoint->load_absolute_joint_trajectory_pose(coordinates1);
	}

	if (track) {
		coordinates2[0] = 0.0;
		coordinates2[1] = -0.087;
		coordinates2[2] = -1.542;
		coordinates2[3] = -0.044;
		coordinates2[4] = 1.109;
		coordinates2[5] = 4.173;
		coordinates2[6] = -2.664;
		sgenjoint->load_absolute_joint_trajectory_pose(coordinates2);
	} else if (postument) {
		coordinates1[0] = -0.104;
		coordinates1[1] = -1.342;
		coordinates1[2] = 0.020;
		coordinates1[3] = 1.034;
		coordinates1[4] = 3.458;
		coordinates1[5] = -2.738;
		sgenjoint->load_absolute_joint_trajectory_pose(coordinates1);
	}

	if (track) {
		coordinates2[0] = 0.0;
		coordinates2[1] = -0.087;
		coordinates2[2] = -1.542;
		coordinates2[3] = 0.024;
		coordinates2[4] = 1.219;
		coordinates2[5] = 4.173;
		coordinates2[6] = -2.664;
		sgenjoint->load_absolute_joint_trajectory_pose(coordinates2);
	} else if (postument) {
		coordinates1[0] = -0.104;
		coordinates1[1] = -1.542;
		coordinates1[2] = 0.020;
		coordinates1[3] = 1.134;
		coordinates1[4] = 3.658;
		coordinates1[5] = -2.738;
		sgenjoint->load_absolute_joint_trajectory_pose(coordinates1);
	}

	if (sgenjoint->calculate_interpolate() && sgenjoint->detect_jerks(1) == 0) {
		sgenjoint->Move();
	}
	// JOINT ABSOLUTE END


	// JOINT RELATIVE
	sr_ecp_msg.message("Joint relative");
	sgenjoint->reset();
	sgenjoint->set_relative();
	if (track) {
		coordinates2[0] = 0.0;
		coordinates2[1] = 0.1;
		coordinates2[2] = 0.0;
		coordinates2[3] = 0.0;
		coordinates2[4] = 0.1;
		coordinates2[5] = 0.0;
		coordinates2[6] = 0.0;
		sgenjoint->load_relative_joint_trajectory_pose(coordinates2);
	} else if (postument) {
		coordinates1[0] = 0.1;
		coordinates1[1] = 0.0;
		coordinates1[2] = 0.0;
		coordinates1[3] = 0.0;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
		sgenjoint->load_relative_joint_trajectory_pose(coordinates1);
	}

	if (track) {
		coordinates2[0] = 0.0;
		coordinates2[1] = -0.2;
		coordinates2[2] = 0.0;
		coordinates2[3] = 0.1;
		coordinates2[4] = 0.0;
		coordinates2[5] = 0.1;
		coordinates2[6] = 0.0;
		sgenjoint->load_relative_joint_trajectory_pose(coordinates2);
	} else if (postument) {
		coordinates1[0] = 0.2;
		coordinates1[1] = 0.1;
		coordinates1[2] = -0.1;
		coordinates1[3] = 0.0;
		coordinates1[4] = 0.6;
		coordinates1[5] = 0.0;
		sgenjoint->load_relative_joint_trajectory_pose(coordinates1);
	}

	if (track) {
		coordinates2[0] = 0.0;
		coordinates2[1] = 0.1;
		coordinates2[2] = 0.0;
		coordinates2[3] = 0.1;
		coordinates2[4] = 0.1;
		coordinates2[5] = -0.1;
		coordinates2[6] = 0.0;
		sgenjoint->load_relative_joint_trajectory_pose(coordinates2);
	} else if (postument) {
		coordinates1[0] = -0.3;
		coordinates1[1] = -0.1;
		coordinates1[2] = 0.1;
		coordinates1[3] = 0.0;
		coordinates1[4] = -0.6;
		coordinates1[5] = 0.0;
		sgenjoint->load_relative_joint_trajectory_pose(coordinates1);
	}

	if (sgenjoint->calculate_interpolate() && sgenjoint->detect_jerks(1) == 0) {
		sgenjoint->Move();
	}
	// JOINT RELATIVE END


	// MOTOR ABSOLUTE
	sr_ecp_msg.message("Motor absolute");
	sgenmotor->reset();
	sgenmotor->set_absolute();
	if (track) {
		coordinates2[0] = 0.0;
		coordinates2[1] = 20.0;
		coordinates2[2] = 5.0;
		coordinates2[3] = 0.0;
		coordinates2[4] = 20.0;
		coordinates2[5] = 0.0;
		coordinates2[6] = 0.0;
		sgenmotor->load_absolute_motor_trajectory_pose(coordinates2);
	} else if (postument) {
		coordinates1[0] = 0.0;
		coordinates1[1] = 20.0;
		coordinates1[2] = 0.0;
		coordinates1[3] = 0.0;
		coordinates1[4] = 20.0;
		coordinates1[5] = 0.0;
		sgenmotor->load_absolute_motor_trajectory_pose(coordinates1);
	}

	if (track) {
		coordinates2[0] = 0.0;
		coordinates2[1] = 10.0;
		coordinates2[2] = 10.0;
		coordinates2[3] = 0.0;
		coordinates2[4] = 0.0;
		coordinates2[5] = 20.0;
		coordinates2[6] = 0.0;
		sgenmotor->load_absolute_motor_trajectory_pose(coordinates2);
	} else if (postument) {
		coordinates1[0] = 0.0;
		coordinates1[1] = 0.0;
		coordinates1[2] = 0.0;
		coordinates1[3] = 50.0;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
		sgenmotor->load_absolute_motor_trajectory_pose(coordinates1);
	}

	if (track) {
		coordinates2[0] = 0.0;
		coordinates2[1] = 0.0;
		coordinates2[2] = 0.0;
		coordinates2[3] = 0.0;
		coordinates2[4] = 0.0;
		coordinates2[5] = 0.0;
		coordinates2[6] = 0.0;
		sgenmotor->load_absolute_motor_trajectory_pose(coordinates2);
	} else if (postument) {
		coordinates1[0] = 0.0;
		coordinates1[1] = -10.0;
		coordinates1[2] = 0.0;
		coordinates1[3] = 0.0;
		coordinates1[4] = 15.0;
		coordinates1[5] = 0.0;
		sgenmotor->load_absolute_motor_trajectory_pose(coordinates1);
	}

	if (sgenmotor->calculate_interpolate() && sgenmotor->detect_jerks(6) == 0) {
		sgenmotor->Move();
	}
	// MOTOR ABSOLUTE END


	// MOTOR RELATIVE
	sr_ecp_msg.message("Motor relative");
	sgenmotor->reset();
	sgenmotor->set_relative();
	if (track) {
		coordinates2[0] = 0.0;
		coordinates2[1] = 20.0;
		coordinates2[2] = 0.0;
		coordinates2[3] = 0.0;
		coordinates2[4] = 20.0;
		coordinates2[5] = 0.0;
		coordinates2[6] = 0.0;
		sgenmotor->load_relative_motor_trajectory_pose(coordinates2);
	} else if (postument) {
		coordinates1[0] = 0.0;
		coordinates1[1] = 20.0;
		coordinates1[2] = 0.0;
		coordinates1[3] = 0.0;
		coordinates1[4] = 20.0;
		coordinates1[5] = 0.0;
		sgenmotor->load_relative_motor_trajectory_pose(coordinates1);
	}

	if (track) {
		coordinates2[0] = 0.0;
		coordinates2[1] = -10.0;
		coordinates2[2] = 0.0;
		coordinates2[3] = 20.0;
		coordinates2[4] = -20.0;
		coordinates2[5] = 0.0;
		coordinates2[6] = 0.0;
		sgenmotor->load_relative_motor_trajectory_pose(coordinates2);
	} else if (postument) {
		coordinates1[0] = 0.0;
		coordinates1[1] = -10.0;
		coordinates1[2] = 0.0;
		coordinates1[3] = 20.0;
		coordinates1[4] = -20.0;
		coordinates1[5] = 0.0;
		sgenmotor->load_relative_motor_trajectory_pose(coordinates1);
	}

	if (track) {
		coordinates2[0] = 0.0;
		coordinates2[1] = -10.0;
		coordinates2[2] = 0.0;
		coordinates2[3] = -10.0;
		coordinates2[4] = 0.0;
		coordinates2[5] = 0.0;
		coordinates2[6] = 0.0;
		sgenmotor->load_relative_motor_trajectory_pose(coordinates2);
	} else if (postument) {
		coordinates1[0] = 0.0;
		coordinates1[1] = -10.0;
		coordinates1[2] = 0.0;
		coordinates1[3] = -20.0;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
		sgenmotor->load_relative_motor_trajectory_pose(coordinates1);
	}

	if (sgenmotor->calculate_interpolate() && sgenmotor->detect_jerks(6) == 0) {
		sgenmotor->Move();
	}
	// MOTOR RELATIVE END


	// EULER ABSOLUTE
	sr_ecp_msg.message("Euler absolute");
	sgeneuler->reset();
	sgeneuler->set_absolute();

	if (track) {
		coordinates1[0] = 0.534991;
		coordinates1[1] = -0.136314;
		coordinates1[2] = 0.186314;
		coordinates1[3] = -2.602063;
		coordinates1[4] = 1.871544;
		coordinates1[5] = 2.204729;
	} else if (postument) {
		coordinates1[0] = 0.529991;
		coordinates1[1] = 1.706314;
		coordinates1[2] = 0.178314;
		coordinates1[3] = -2.185063;
		coordinates1[4] = 1.666544;
		coordinates1[5] = 2.328729;
	}
	sgeneuler->load_absolute_euler_zyz_trajectory_pose(coordinates1);

	if (track) {
		coordinates1[0] = 0.534991;
		coordinates1[1] = -0.156314;
		coordinates1[2] = 0.186314;
		coordinates1[3] = -2.662063;
		coordinates1[4] = 1.871544;
		coordinates1[5] = 2.234729;
	} else if (postument) {
		coordinates1[0] = 0.529991;
		coordinates1[1] = 1.7506314;
		coordinates1[2] = 0.178314;
		coordinates1[3] = -2.185063;
		coordinates1[4] = 1.566544;
		coordinates1[5] = 2.328729;
	}
	sgeneuler->load_absolute_euler_zyz_trajectory_pose(coordinates1);

	if (track) {
		coordinates1[0] = 0.534991;
		coordinates1[1] = -0.176314;
		coordinates1[2] = 0.186314;
		coordinates1[3] = -2.662063;
		coordinates1[4] = 1.871544;
		coordinates1[5] = 2.234729;
	} else if (postument) {
		coordinates1[0] = 0.529991;
		coordinates1[1] = 1.806314;
		coordinates1[2] = 0.178314;
		coordinates1[3] = -2.185063;
		coordinates1[4] = 1.766544;
		coordinates1[5] = 2.328729;
	}
	sgeneuler->load_absolute_euler_zyz_trajectory_pose(coordinates1);

	if (sgeneuler->calculate_interpolate() && sgeneuler->detect_jerks(0.3) == 0) {
		sgeneuler->Move();
	}
	// EULER ABSOLUTE END


	// EULER RELATIVE
	sr_ecp_msg.message("Euler relative");
	sgeneuler->reset();
	sgeneuler->set_relative();

	if (track) {
		coordinates1[0] = 0.0;
		coordinates1[1] = 0.1;
		coordinates1[2] = 0.0;
		coordinates1[3] = 0.15;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
	} else if (postument) {
		coordinates1[0] = 0.0;
		coordinates1[1] = 0.1;
		coordinates1[2] = 0.0;
		coordinates1[3] = 0.15;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
	}
	sgeneuler->load_relative_euler_zyz_trajectory_pose(coordinates1);

	if (track) {
		coordinates1[0] = 0.0;
		coordinates1[1] = -0.1;
		coordinates1[2] = 0.1;
		coordinates1[3] = -0.15;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
	} else if (postument) {
		coordinates1[0] = 0.0;
		coordinates1[1] = -0.1;
		coordinates1[2] = 0.1;
		coordinates1[3] = -0.15;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
	}
	sgeneuler->load_relative_euler_zyz_trajectory_pose(coordinates1);

	if (track) {
		coordinates1[0] = 0.0;
		coordinates1[1] = 0.0;
		coordinates1[2] = -0.1;
		coordinates1[3] = 0.0;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
	} else if (postument) {
		coordinates1[0] = 0.0;
		coordinates1[1] = 0.0;
		coordinates1[2] = -0.1;
		coordinates1[3] = 0.0;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
	}
	sgeneuler->load_relative_euler_zyz_trajectory_pose(coordinates1);

	if (sgeneuler->calculate_interpolate() && sgeneuler->detect_jerks(0.3) == 0) {
		sgeneuler->Move();
	}
	// EULER RELATIVE END


	// ANGLE AXIS ABSOLUTE
	sr_ecp_msg.message("Angle axis absolute");
	sgenangle->reset();
	sgenangle->set_absolute();

	if (track) {
		coordinates1[0] = 0.534987;
		coordinates1[1] = -0.176317;
		coordinates1[2] = 0.180306;
		coordinates1[3] = 1.000713;
		coordinates1[4] = -1.447889;
		coordinates1[5] = -0.264191;
	} else if (postument) {
		coordinates1[0] = 0.529987;
		coordinates1[1] = 1.606317;
		coordinates1[2] = 0.178306;
		coordinates1[3] = 1.267713;
		coordinates1[4] = -1.219889;
		coordinates1[5] = 0.174191;
	}
	sgenangle->load_absolute_angle_axis_trajectory_pose(coordinates1);

	if (track) {
		coordinates1[0] = 0.504987;
		coordinates1[1] = -0.146317;
		coordinates1[2] = 0.106306;
		coordinates1[3] = 1.103713;
		coordinates1[4] = -1.407889;
		coordinates1[5] = -0.294191;
	} else if (postument) {
		coordinates1[0] = 0.509987;
		coordinates1[1] = 1.806317;
		coordinates1[2] = 0.148306;
		coordinates1[3] = 1.367713;
		coordinates1[4] = -1.119889;
		coordinates1[5] = 0.114191;
	}
	sgenangle->load_absolute_angle_axis_trajectory_pose(coordinates1);

	if (track) {
		coordinates1[0] = 0.534987;
		coordinates1[1] = -0.176317;
		coordinates1[2] = 0.186306;
		coordinates1[3] = 1.203713;
		coordinates1[4] = -1.447889;
		coordinates1[5] = -0.294191;
	} else if (postument) {
		coordinates1[0] = 0.529987;
		coordinates1[1] = 1.806317;
		coordinates1[2] = 0.178306;
		coordinates1[3] = 1.367713;
		coordinates1[4] = -1.119889;
		coordinates1[5] = 0.104191;
	}
	sgenangle->load_absolute_angle_axis_trajectory_pose(coordinates1);

	if (sgenangle->calculate_interpolate() && sgeneuler->detect_jerks(0.3) == 0) {
		sgenangle->detect_jerks(0.3);
		sgenangle->Move();
	}
	// ANGLE AXIS ABSOLUTE END


	// ANGLE AXIS RELATIVE
	sr_ecp_msg.message("Angle axis relative");
	sgenangle->reset();
	sgenangle->set_relative();

	if (track) {
		coordinates1[0] = 0.0;
		coordinates1[1] = 0.1;
		coordinates1[2] = 0.0;
		coordinates1[3] = 0.05;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
	} else if (postument) {
		coordinates1[0] = 0.0;
		coordinates1[1] = 0.1;
		coordinates1[2] = 0.0;
		coordinates1[3] = 0.05;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
	}
	sgenangle->load_relative_angle_axis_trajectory_pose(coordinates1);

	if (track) {
		coordinates1[0] = 0.0;
		coordinates1[1] = -0.1;
		coordinates1[2] = 0.1;
		coordinates1[3] = -0.05;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
	} else if (postument) {
		coordinates1[0] = 0.0;
		coordinates1[1] = -0.1;
		coordinates1[2] = 0.1;
		coordinates1[3] = -0.05;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
	}
	sgenangle->load_relative_angle_axis_trajectory_pose(coordinates1);

	if (track) {
		coordinates1[0] = 0.0;
		coordinates1[1] = 0.0;
		coordinates1[2] = -0.1;
		coordinates1[3] = 0.0;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
	} else if (postument) {
		coordinates1[0] = 0.0;
		coordinates1[1] = 0.0;
		coordinates1[2] = -0.1;
		coordinates1[3] = 0.0;
		coordinates1[4] = 0.0;
		coordinates1[5] = 0.0;
	}
	sgenangle->load_relative_angle_axis_trajectory_pose(coordinates1);

	if (sgenangle->calculate_interpolate() && sgenangle->detect_jerks(0.3) == 0) {
		sgenangle->Move();
	}
	// ANGLE AXIS RELATIVE END
}

sub_task_smooth_gen_test::~sub_task_smooth_gen_test()
{
	delete sgenjoint;
	delete sgenmotor;
	delete sgeneuler;
	delete sgenangle;
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
