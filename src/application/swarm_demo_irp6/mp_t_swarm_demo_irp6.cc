#include "mp_t_swarm_demo_irp6.h"

// ecp generators to be commanded
#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/smooth_file_from_mp/ecp_mp_g_smooth_file_from_mp.h"
#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"

// mp_robots headers
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"

namespace mrrocpp {
namespace mp {
namespace task {

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void swarm_demo::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_m);
}

swarm_demo::swarm_demo(lib::configurator &_config) :
		task(_config)
{
}

void swarm_demo::main_task_algorithm(void)
{
	sr_ecp_msg->message("Swarm Demo irp6 (MP) START");

	sr_ecp_msg->message("Both Joint");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/swarm_demo_irp6/trajectory_track_joint.trj", lib::irp6ot_m::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/swarm_demo_irp6/trajectory_postument_joint.trj", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME, lib::irp6p_m::ROBOT_NAME);

	sr_ecp_msg->message("Both Bias");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE, 0, "", lib::irp6ot_m::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE, 0, "", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME, lib::irp6p_m::ROBOT_NAME);

	sr_ecp_msg->message("Postument Angle axis");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 0, "../../src/application/swarm_demo_irp6/trajectory_postument_angle_p1.trj", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	sr_ecp_msg->message("Postument Force approach");
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.02, 300, 3), lib::irp6p_m::ROBOT_NAME);

	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	sr_ecp_msg->message("Track Joint");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/swarm_demo_irp6/trajectory_track_joint_p1.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	sr_ecp_msg->message("Track angle axis");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 0, "../../src/application/swarm_demo_irp6/trajectory_track_angle_p1a.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	sr_ecp_msg->message("Track Force approach");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.02, 300, 3), lib::irp6ot_m::ROBOT_NAME);

	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	sr_ecp_msg->message("Postument angle axis2");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 0, "../../src/application/swarm_demo_irp6/trajectory_postument_angle_p2.trj", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	sr_ecp_msg->message("Postument Force approach");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.02, 300, 3), lib::irp6p_m::ROBOT_NAME);

	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	sr_ecp_msg->message("Track angle axis2");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 0, "../../src/application/swarm_demo_irp6/trajectory_track_angle_p2.trj", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	sr_ecp_msg->message("Track Force approach");
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.02, 300, 3), lib::irp6ot_m::ROBOT_NAME);

	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	sr_ecp_msg->message("Wait");

	wait_ms(2000);

	sr_ecp_msg->message("Both angle axis");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 0, "../../src/application/swarm_demo_irp6/trajectory_track_angle.trj", lib::irp6ot_m::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 0, "../../src/application/swarm_demo_irp6/trajectory_postument_angle.trj", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME, lib::irp6p_m::ROBOT_NAME);

	sr_ecp_msg->message("Swarm Demo END");

}

task* return_created_mp_task(lib::configurator &_config)
{
	return new swarm_demo(_config);
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
