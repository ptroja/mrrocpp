#include "ecp_t_irp6_m.h"

// generators to be register headers
#include "generator/ecp/smooth_file_from_mp/ecp_g_smooth_file_from_mp.h"
#include "generator/ecp/bias_edp_force/ecp_g_bias_edp_force.h"
#include "generator/ecp/tff_nose_run/ecp_g_tff_nose_run.h"
#include "generator/ecp/sleep/ecp_g_sleep.h"
#include "generator/ecp/transparent/ecp_g_transparent.h"
#include "generator/ecp/tff_rubik_face_rotate/ecp_g_tff_rubik_face_rotate.h"
#include "generator/ecp/tff_gripper_approach/ecp_g_tff_gripper_approach.h"
#include "generator/ecp/weight_measure/ecp_g_weight_measure.h"

// ecp_robots headers
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

irp6_m::irp6_m(lib::configurator &_config) :
		common::task::task(_config)
{
	// the robot is choose dependently on the section of configuration file sent as argv[4]
	if (config.robot_name == lib::irp6ot_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6ot_m::robot(*this);
	} else if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_m::robot(*this);
	} else {
		throw std::runtime_error("Robot not supported");
	}

	// generator registration
	register_generator(new generator::sleep(*this));
	register_generator(new generator::transparent(*this));
	register_generator(new generator::bias_edp_force(*this));
	register_generator(new generator::tff_gripper_approach(*this, 8));
	register_generator(new generator::tff_rubik_face_rotate(*this, 8));
	register_generator(new generator::tff_nose_run(*this, 8));
	register_generator(new generator::weight_measure(*this, 1));
	register_generator(new generator::smooth_file_from_mp(*this, lib::ECP_JOINT, ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, true));
	register_generator(new generator::smooth_file_from_mp(*this, lib::ECP_XYZ_ANGLE_AXIS, ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, true));

	sr_ecp_msg->message("ecp loaded");
}

}
}

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::irp6_m(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
