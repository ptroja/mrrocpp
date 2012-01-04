#include <cmath>
#include <fstream>

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"

#include "BReading.h"
#include "ecp_t_block_move.h"

#include "subtask/ecp_st_smooth_file_from_mp.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"
#include "subtask/ecp_st_bias_edp_force.h"
//#include "subtask/ecp_mp_st_gripper_opening.h"

#include "generator/ecp/force/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"
#include "generator/ecp/ecp_g_newsmooth.h"
#include "generator/ecp/ecp_g_multiple_position.h"

#include "sensor/discode/discode_sensor.h"
#include "../visual_servoing/visual_servoing.h"

#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
//#include "robot/irp6p_tfg/const_irp6p_tfg.h"

#define BOARD_COLOR 5

#define BLOCK_WIDTH 0.0325
#define BLOCK_HEIGHT 0.0193

#define BLOCK_REACHING 0
#define BUILDING 1

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
		ecp_m_robot = shared_ptr <robot_t> (new irp6p_m::robot(*this));
	} else {
		throw std::runtime_error("Robot not supported");
	}

	log_dbg_enabled = true;

	// utworzenie generatorow
	gtga = new common::generator::tff_gripper_approach(*this, 8);
	sg = new common::generator::newsmooth(*this,lib::ECP_XYZ_ANGLE_AXIS, 6);
	gp = new common::generator::get_position(*this,lib::ECP_XYZ_ANGLE_AXIS, 6);
	//stgo = new common::sub_task::gripper_opening(*this);

	// utworzenie podzadan
	subtask_m[ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE] = new sub_task::bias_edp_force(*this);;
	subtask_m[ecp_mp::sub_task::ECP_ST_SMOOTH_JOINT_FILE_FROM_MP] = new sub_task::sub_task_smooth_file_from_mp(*this, lib::ECP_JOINT, true);
	subtask_m[ecp_mp::sub_task::ECP_ST_SMOOTH_ANGLE_AXIS_FILE_FROM_MP] = new sub_task::sub_task_smooth_file_from_mp(*this, lib::ECP_XYZ_ANGLE_AXIS, true);

	//sensor rpc
	sr_ecp_msg->message("Creating discode sensor...");
	ds_config_section_name = "[discode_sensor]";
	ds_rpc = shared_ptr <discode_sensor> (new discode_sensor(config, ds_config_section_name));
	ds_rpc->configure_sensor();

	//int tm1 = config.value <int> ("sm1_timeout", "[ecp_block_move]");

	//get position compute parameters
	ecp_bm_config_section_name = "[ecp_block_move]";
	offset = config.value <6, 1> ("offset", ecp_bm_config_section_name);
	block_size = config.value <6, 1> ("block_size", ecp_bm_config_section_name);
	correction = config.value <6, 1> ("correction", ecp_bm_config_section_name);
	position = config.value <6, 1> ("position", ecp_bm_config_section_name);
	int tm2 = config.value <int> ("sm2_timeout", ecp_bm_config_section_name);

/*
	//board localization servovision
	sr_ecp_msg->message("Creating visual servo 1...");
	vs_config_section_name1 = "[board_localization_servovision]";
	shared_ptr <position_constraint> cube1(new cubic_constraint(config, vs_config_section_name1));
	reg1 = shared_ptr <visual_servo_regulator> (new regulator_p(config, vs_config_section_name1));
	ds1 = shared_ptr <discode_sensor> (new discode_sensor(config, vs_config_section_name1));
	vs1 = shared_ptr <visual_servo> (new ib_eih_visual_servo(reg1, ds1, vs_config_section_name1, config));
	object_reached_term_cond1 = shared_ptr <termination_condition> (new object_reached_termination_condition(config, vs_config_section_name1));
	timeout_term_cond1 = shared_ptr <termination_condition> (new timeout_termination_condition(tm1));

	//utworzenie generatora ruchu
	sm1 = shared_ptr <single_visual_servo_manager> (new single_visual_servo_manager(*this, vs_config_section_name1.c_str(), vs1));
	sm1->add_position_constraint(cube1);
	sm1->configure();
*/
	//block reaching servovision
	sr_ecp_msg->message("Creating visual servo 2...");
	vs_config_section_name2 = "[block_reaching_servovision]";
	shared_ptr <position_constraint> cube2(new cubic_constraint(config, vs_config_section_name2));
	reg2 = shared_ptr <visual_servo_regulator> (new regulator_p(config, vs_config_section_name2));
	ds2 = shared_ptr <discode_sensor> (new discode_sensor(config, vs_config_section_name2));
	vs2 = shared_ptr <visual_servo> (new ib_eih_visual_servo(reg2, ds2, vs_config_section_name2, config));
	object_reached_term_cond2 = shared_ptr <termination_condition> (new object_reached_termination_condition(config, vs_config_section_name2));
	timeout_term_cond2 = shared_ptr <termination_condition> (new timeout_termination_condition(tm2));

	//utworzenie generatora ruchu
	sm2 = shared_ptr <single_visual_servo_manager> (new single_visual_servo_manager(*this, vs_config_section_name2.c_str(), vs2));
	sm2->add_position_constraint(cube2);
	sm2->configure();

	sr_ecp_msg->message("ecp BLOCK MOVE loaded");
}

void block_move::mp_2_ecp_next_state_string_handler(void)
{
	if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH) {

		sr_ecp_msg->message("configurate tff_gripper_approach...");

		switch(mp_command.ecp_next_state.variant) {

			case BLOCK_REACHING:
				sr_ecp_msg->message("reaching the block...");
				gtga->configure(0.03, 800, 3);
				gtga->Move();
				break;

			case BUILDING:
				sr_ecp_msg->message("reaching the platform...");
				gtga->configure(0.02, 600, 2);
				gtga->Move();
				break;

			default:
				sr_ecp_msg->message("unknown task for tff_gripper_approach...");
				break;
		}

		sr_ecp_msg->message("tff_gripper_approach end");
	}
	else if(mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST) {

		sr_ecp_msg->message("configurate sensor...");

		uint32_t param = (int) mp_command.ecp_next_state.variant;

		Types::Mrrocpp_Proxy::BReading br;

		br = ds_rpc->call_remote_procedure<Types::Mrrocpp_Proxy::BReading>((int) param);

		if(br.rpcReceived) {
			sr_ecp_msg->message("Rpc received");
		}

		sr_ecp_msg->message("configurate servovision...");

/*		if(param == BOARD_COLOR) {

			sm1_move();

			//obsługa warunku zakończenia pracy - warunek stopu
			if(object_reached_term_cond1->is_condition_met()) {
				sr_ecp_msg->message("object_reached_term_cond is met");
				ecp_reply.recognized_command[0] = 'Y';

				//odczyt pozycji w ANGLE_AXIS
				gp->Move();
				position_vector = gp->get_position_vector();

				if(!position.size()) {
					sr_ecp_msg->message("get_position_vector is empty");
				}

				std::cout << "POSITION" << endl;
				for(size_t i = 0; i < position_vector.size(); ++i) {
					std::cout << position_vector[i] << std::endl;
				}
				std::cout << std::endl;
			}
			else {
				sr_ecp_msg->message("object_reached_term_cond IS NOT MET");
			}

			//obsługa warunku zakończenia pracy - timeout
			if(timeout_term_cond1->is_condition_met()) {
				sr_ecp_msg->message("timeout_term_cond is met");
				ecp_reply.recognized_command[0] = 'N';
			}
			else {
				sr_ecp_msg->message("timeout_term_cond IS NOT MET");
			}
		}
		else {
*/
			sm2->add_termination_condition(object_reached_term_cond2);
			sm2->add_termination_condition(timeout_term_cond2);
			sm2->Move();

			//obsługa warunku zakończenia pracy - warunek stopu
			if(object_reached_term_cond2->is_condition_met()) {
				sr_ecp_msg->message("object_reached_term_cond is met");
				ecp_reply.recognized_command[0] = 'Y';
			}
			else {
				sr_ecp_msg->message("object_reached_term_cond IS NOT MET");
			}

			//obsługa warunku zakończenia pracy - timeout
			if(timeout_term_cond2->is_condition_met()) {
				sr_ecp_msg->message("timeout_term_cond is met");
				ecp_reply.recognized_command[0] = 'N';
			}
			else {
				sr_ecp_msg->message("timeout_term_cond IS NOT MET");
			}
//		}

		sr_ecp_msg->message("servovision end");
	}
	else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_NEWSMOOTH) {

		sr_ecp_msg->message("configurate Smooth Generator...");

		int param = (int) mp_command.ecp_next_state.variant;

		sr_ecp_msg->message("after loading param from variant");

		int change_pos[6];

		change_pos[2] = param % 10;
		change_pos[1] = (param % 100 - change_pos[2])/10;
		change_pos[0] = (param % 1000 - change_pos[1])/100;
		change_pos[3] = 0;
		change_pos[4] = 0;
		change_pos[5] = 0;

		std::cout << "CHANGE_POSITION" << endl;
		for(size_t i = 0; i < 6; ++i) {
			std::cout << change_pos[i] << std::endl;
		}
		std::cout << std::endl;

		position_on_board(0,0) = change_pos[0] - 1.0;
		position_on_board(1,0) = change_pos[1] - 1.0;
		position_on_board(2,0) = change_pos[2] - 2.0;
		position_on_board(3,0) = 0;
		position_on_board(4,0) = 0;
		position_on_board(5,0) = 0;

		correction_weights(0,0) = (position_on_board(0,0) == 0.0) ? 0 : 1;
		correction_weights(1,0) = (position_on_board(1,0) == 0.0) ? 0 : 1;
		correction_weights(2,0) = 0;
		correction_weights(3,0) = 0;
		correction_weights(4,0) = 0;
		correction_weights(5,0) = 0;

		sr_ecp_msg->message("after load coordinates");

		int do_move = 0;	//czy zmieniac pozycje
		for(int i = 0; i < 6; ++i) {
			if(abs(position_on_board(i,0)) < 4 && position_on_board(i,0) >= 0) {
				do_move = 1;
			}
		}

		if(do_move == 1)
		{
			sg->reset();
			sg->set_absolute();

			std::vector <double> coordinates_vector(6);

			sr_ecp_msg->message("after reset and set_absoulute");

			std::cout << "POSITION ON BOARD" << endl;
			for(size_t i = 0; i < 6; ++i) {
				std::cout << position_on_board(i,0) << std::endl;
			}
			std::cout << std::endl;


			/*coordinates[0] = position[0] + offset_x + (4 - change_pos[0])*BLOCK_WIDTH + corr_x*(change_pos[0] - 1);
			coordinates[1] = position[1] + offset_y + (4 - change_pos[1])*BLOCK_WIDTH + corr_y*(change_pos[0] - 1);
			coordinates[2] = position[2] + (change_pos[2]-3)*BLOCK_HEIGHT;
			coordinates[3] = position[3] + change_pos[3]*BLOCK_WIDTH;
			coordinates[4] = position[4] + change_pos[4]*BLOCK_WIDTH;
			coordinates[5] = position[5] + change_pos[5]*BLOCK_WIDTH;
			*/

			for(size_t i = 0; i < 6; ++i) {
				coordinates_vector[i] = position(i) + offset(i,0) + position_on_board(i,0)*block_size(i,0) + correction(i,0)*correction_weights(i,0);
			}

			/*
			std::cout << "coordinates: " << std::endl;
			for(int i = 0; i < 6; ++i) {
				std::cout << coordinates[i] << std::endl;
			}
			*/

			sr_ecp_msg->message("coordinates ready");

			sg->load_absolute_angle_axis_trajectory_pose(coordinates_vector);

			sr_ecp_msg->message("pose loaded");

			if(sg->calculate_interpolate())
			{
				sg->Move();
			}

			sr_ecp_msg->message("smooth generator configuration end");

		}

	}

}

void block_move::sm1_move(void) {
	//sm1->add_termination_condition(object_reached_term_cond1);
	//sm1->add_termination_condition(timeout_term_cond1);
	//sm1->Move();
}

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::block_move(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
