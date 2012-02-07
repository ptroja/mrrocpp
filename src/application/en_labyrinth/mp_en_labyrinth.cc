
#include <cstdio>
#include <unistd.h>
#include <string.h>
#include <cstring>
#include <iostream>

#include "mp_en_labyrinth.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"


#include "base/mp/mp_task.h"
//#include "base/mp/MP_main_error.h"
//#include "../edge_follow/mp_t_edge_follow_mr.h"
#include "base/lib/mrmath/mrmath.h"

#include "robot/irp6_tfg/dp_tfg.h"

#include "application/edge_follow/ecp_mp_st_edge_follow.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"
#include "subtask/ecp_mp_st_tff_nose_run.h"
#include "generator/ecp/ecp_mp_g_tfg.h"

#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"

#include "ecp_mp_g_g_en_labyrinth.h"
//#include "ecp_mp_g_g_rotate_en_labyrinth.h"

#include "sensor/discode/discode_sensor.h"
#include <boost/shared_ptr.hpp>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "EN_Labyrinth_Reading.hpp"

#define K_MAX 0.0365

using mrrocpp::ecp_mp::sensor::discode::discode_sensor;

using namespace std;

namespace mrrocpp {
namespace mp {
namespace task {
task* return_created_mp_task(lib::configurator &_config)
{
	return new mp_en_labyrinth(_config);
}

void mp_en_labyrinth::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(irp6p_tfg);
}

mp_en_labyrinth::mp_en_labyrinth(lib::configurator &_config) :
		task(_config)
{
	sr_ecp_msg->message("EN_Labirynth");

	char config_section_name[] = { "[EN_Labirynth]" };
	discode = boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor>(new mrrocpp::ecp_mp::sensor::discode::discode_sensor(config, config_section_name));

	sr_ecp_msg->message("Configuring DisCODe sensor");
	discode->configure_sensor();

	ERROR = false;
}

void mp_en_labyrinth::main_task_algorithm(void)
{
	sr_ecp_msg->message("mp start");

	Types::Mrrocpp_Proxy::EN_Labyrinth_Reading reading;

	sr_ecp_msg->message("reading init");
	reading = discode->call_remote_procedure<Types::Mrrocpp_Proxy::EN_Labyrinth_Reading>(double(29.0386));
	sr_ecp_msg->message("reading received");

//	while(1)
//	{

//		if(!reading.labyrinth_solved)
//			continue;

		cout << "Reading info: " << endl;
		cout << "Solved: " << reading.labyrinth_solved << endl;
		cout << "Path Size: " << reading.path_size << endl;
		cout << "Start_pt: (" << reading.start_point_x << "," << reading.start_point_y << ")" << endl;
		cout << "End_pt ("  << reading.end_point_x << "," << reading.end_point_y << ")" << endl;
		cout << "Path: ";
		for(int i=0; i<reading.path_size; ++i)
			cout << reading.path[i] << " ";
		cout << endl;


//		// TODO better read the data then make up...
//		bool labyrinth_solved_static = true;
//		int path_size_static = 10;
//		int start_point_x_static = 1;
//		int start_point_y_static = 1;
//		int end_point_x_static = 8;
//		int end_point_y_static = 8;
//		int path_static[10] = {0, 1, 2, 3, 0, 1, 2, 3, 0, 1};
//
		lib::robot_name_t manipulator_name;
		lib::robot_name_t gripper_name;

		manipulator_name = lib::irp6p_m::ROBOT_NAME;
		gripper_name = lib::irp6p_tfg::ROBOT_NAME;

		sr_ecp_msg->message("Moving to first point");

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "ABSOLUTE_JOIN 1.117 -1.636 0.11 -0.041 4.669 -1.816", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());


		sr_ecp_msg->message("Moving to the ending point");
		cout << "Ending point: (" << reading.end_point_x << "," << reading.end_point_y << ")" << endl;
		std::string relative_move;
		bool error = false;
		for(int i=0; i<reading.path_size && !error; ++i)
		{
			cout << "Point " << i << " is " << reading.path[i] << endl;
			switch(reading.path[i])
			{
				case UP:
					relative_move = "RELATIVE_EULER 0.02 0.0 0.0 0.0 0.0 0.0";
				break;
				case DOWN:
					relative_move = "RELATIVE_EULER -0.02 0.0 0.0 0.0 0.0 0.0";
				break;
				case LEFT:
					relative_move = "RELATIVE_EULER 0.0 0.02 0.0 0.0 0.0 0.0";
				break;
				case RIGHT:
					relative_move = "RELATIVE_EULER 0.0 -0.02 0.0 0.0 0.0 0.0";
				break;
				default:
					error = true;
					sr_ecp_msg->message("Path reading from DisCODe contains error in direction description!");
				break;
			}
			set_next_ecp_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, relative_move, lib::irp6p_m::ROBOT_NAME);
			wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
		}

//		char data[64];
//		sprintf (data, "%i %lf", 0, K_MAX);
//		set_next_ecp_state(ecp_mp::generator::ECP_GEN_G_EN_LAB, (int) 5, data, lib::irp6p_m::ROBOT_NAME);
//		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
//
//
//
//		sprintf (data, "%i %lf", 2, K_MAX);
//		set_next_ecp_state(ecp_mp::generator::ECP_GEN_G_EN_LAB, (int) 5, data, lib::irp6p_m::ROBOT_NAME);
//		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());



//		sr_ecp_msg->message("Returning from the labyrinth");
//		set_next_ecp_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/mm_test/w_gore.trj", 0, lib::irp6p_m::ROBOT_NAME);
//		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
//	}

	sr_ecp_msg->message("mp end");
}

} // namespace task
} // namespace mp
} // namespace mrrocpp


