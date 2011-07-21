
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
#include "base/mp/MP_main_error.h"
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


	cout << "Reading info: " << endl;
	cout << "Solved: " << reading.labyrinth_solved << endl;
	cout << "Path Size: " << reading.path_size << endl;
	cout << "Start_pt: (" << reading.start_point_x << "," << reading.start_point_y << ")" << endl;
	cout << "End_pt ("  << reading.start_point_x << "," << reading.start_point_y << ")" << endl;


	// TODO better read the data then make up...
	reading.labyrinth_solved = true;
	reading.path_size = 10;
	reading.start_point_x = 1;
	reading.start_point_y = 1;
	reading.end_point_x = 8;
	reading.end_point_y = 8;
	int path[10] = {2, 2, 2, 0, 2, 0, 2, 0, 2, 0};

	lib::robot_name_t manipulator_name;
	lib::robot_name_t gripper_name;

	manipulator_name = lib::irp6p_m::ROBOT_NAME;
	gripper_name = lib::irp6p_tfg::ROBOT_NAME;


	sr_ecp_msg->message("Moving to the starting position");
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/en_labyrinth/poz_pocz.trj", 0,lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

//	if(reading.start_point_x != 0 && reading.start_point_y != 0)
//	{
//		sr_ecp_msg->message("Moving to the starting point");
//		cout << "Starting point: (" << reading.start_point_x << "," << reading.start_point_y << ")" << endl;
//		for(int i=0; i<reading.start_point_x; ++i)
//		{
//			char data[64];
//			sprintf (data, "%i %lf", RIGHT, K_MAX);
//			set_next_ecp_state(ecp_mp::generator::ECP_GEN_G_EN_LAB, (int) 5, data, 0, lib::irp6p_m::ROBOT_NAME);
//			wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
//		}
//		for(int i=0; i<reading.start_point_y; ++i)
//		{
//			char data[64];
//			sprintf (data, "%i %lf", UP, K_MAX);
//			set_next_ecp_state(ecp_mp::generator::ECP_GEN_G_EN_LAB, (int) 5, data, 0, lib::irp6p_m::ROBOT_NAME);
//			wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
//		}
//	}

	sr_ecp_msg->message("Moving to the ending point");
	cout << "Ending point: (" << reading.end_point_x << "," << reading.end_point_y << ")" << endl;
	for(int i=0; i<reading.path_size; ++i)
	{
		cout << "Point " << i << " is " << path[i] << endl;
		char data[64];
		sprintf (data, "%i %lf", path[i], K_MAX);
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_G_EN_LAB, (int) 5, data, 0, lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
	}

	sr_ecp_msg->message("Returning from the labyrinth");
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/mm_test/w_gore.trj", 0, lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());


	sr_ecp_msg->message("mp end");
}

} // namespace task
} // namespace mp
} // namespace mrrocpp


