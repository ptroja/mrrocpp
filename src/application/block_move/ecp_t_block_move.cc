#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"

#include "ecp_t_block_move.h"

#include "subtask/ecp_st_smooth_file_from_mp.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"
#include "subtask/ecp_st_bias_edp_force.h"
//#include "subtask/ecp_mp_st_gripper_opening.h"

#include "generator/ecp/force/ecp_mp_g_tff_gripper_approach.h"

#include "sensor/discode/discode_sensor.h"

#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
//#include "robot/irp6p_tfg/const_irp6p_tfg.h"
#include "../visual_servoing/visual_servoing.h"

#include "BReading.h"

using namespace mrrocpp::ecp_mp::sensor::discode;
using namespace mrrocpp::ecp::common::generator;
using namespace logger;
using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
block_move::block_move(lib::configurator &_config) :
	common::task::task(_config)
{
	if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_m::robot(*this);
	} else {
		throw std::runtime_error("Robot not supported");
	}

	log_dbg_enabled = true;

	// utworzenie generatorow
	gtga = new common::generator::tff_gripper_approach(*this, 8);
	//stgo = new common::sub_task::gripper_opening(*this);

	// utworzenie podzadan
	subtask_m[ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE] = new sub_task::bias_edp_force(*this);;
	subtask_m[ecp_mp::sub_task::ECP_ST_SMOOTH_JOINT_FILE_FROM_MP] = new sub_task::sub_task_smooth_file_from_mp(*this, lib::ECP_JOINT, true);
	subtask_m[ecp_mp::sub_task::ECP_ST_SMOOTH_ANGLE_AXIS_FILE_FROM_MP] = new sub_task::sub_task_smooth_file_from_mp(*this, lib::ECP_XYZ_ANGLE_AXIS, true);

	//sensor rpc
	sr_ecp_msg->message("Creating discode sensor...");
	ds_config_section_name = "[discode_sensor]";
	ds_rpc = shared_ptr <discode_sensor> (new discode_sensor(config, ds_config_section_name));

	//serwowizja
	sr_ecp_msg->message("Creating visual servo...");
	vs_config_section_name = "[object_follower_ib]";
	shared_ptr <position_constraint> cube(new cubic_constraint(config, vs_config_section_name));
	reg = shared_ptr <visual_servo_regulator> (new regulator_p(config, vs_config_section_name));
	ds = shared_ptr <discode_sensor> (new discode_sensor(config, vs_config_section_name));
	vs = shared_ptr <visual_servo> (new ib_eih_visual_servo(reg, ds, vs_config_section_name, config));
	object_reached_term_cond = shared_ptr <termination_condition> (new object_reached_termination_condition(config, vs_config_section_name));
	timeout_term_cond = shared_ptr <termination_condition> (new timeout_termination_condition(5));

	//utworzenie generatora ruchu
	sm = shared_ptr <single_visual_servo_manager> (new single_visual_servo_manager(*this, vs_config_section_name.c_str(), vs));
	sm->add_position_constraint(cube);

	sr_ecp_msg->message("ecp BLOCK MOVE loaded");
}

void block_move::mp_2_ecp_next_state_string_handler(void)
{
	if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH) {

		sr_ecp_msg->message("configurate tff_gripper_approach...");

		gtga->configure(0.05, 350, 4);
		gtga->Move();

		sr_ecp_msg->message("tff_gripper_approach end");
	}
	else if(mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST) {

		sr_ecp_msg->message("configurate sensor...");

		ds_rpc->configure_sensor();
		uint32_t param = (int) mp_command.ecp_next_state.variant;

		Types::Mrrocpp_Proxy::BReading br;
		br = ds_rpc->call_remote_procedure<Types::Mrrocpp_Proxy::BReading>((int) param);

		sr_ecp_msg->message("configurate servovision...");

		sm->add_termination_condition(object_reached_term_cond);
		sm->add_termination_condition(timeout_term_cond);
		sm->configure();

		sm->Move();

		sr_ecp_msg->message("tff_gripper_approach end");
	}

	//obsługa warunku zakończenia pracy - warunek stopu
	if(object_reached_term_cond->is_condition_met()) {
		sr_ecp_msg->message("object_reached_term_cond is met");
	}
	else {
		sr_ecp_msg->message("object_reached_term_cond IS NOT MET");
	}

	//obsługa warunku zakończenia pracy - timeout
	if(timeout_term_cond->is_condition_met()) {
		sr_ecp_msg->message("timeout_term_cond is met");
	}
	else {
		sr_ecp_msg->message("timeout_term_cond IS NOT MET");
	}

}

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::block_move(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
