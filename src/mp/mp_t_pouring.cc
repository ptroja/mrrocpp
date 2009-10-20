// -------------------------------------------------------------------------
//
// MP Master Process - methods�for pouring task sporadic coordination
//
// -------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_t_pouring.h"

#include <list>
#include <map>

#include "mp/mp_g_force.h"
#include "mp/mp_g_vis.h"
#include "ecp_mp/ecp_mp_s_vis.h"
#include "ecp_mp/ecp_mp_tr_rc_windows.h"

#include <boost/foreach.hpp>

namespace mrrocpp {
namespace mp {
namespace task {

void pouring::approach(void)
{
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_ap.trj", 1, lib::ROBOT_IRP6_ON_TRACK);

	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_ap.trj", 1, lib::ROBOT_IRP6_POSTUMENT);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(2, 2,
			lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT);

}

void pouring::grab(void)
{
	//Ustawienie w pozycji chwytania
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_grab.trj", 1, lib::ROBOT_IRP6_ON_TRACK);

	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_grab.trj", 1, lib::ROBOT_IRP6_POSTUMENT);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(2, 2,
			lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT);
	//Zacisniecie szczek chwytakow
	set_next_ecps_state( (int) ecp_mp::task::GRIP, 0, "", 1, lib::ROBOT_IRP6_ON_TRACK);

	set_next_ecps_state( (int) ecp_mp::task::GRIP, 0, "", 1, lib::ROBOT_IRP6_POSTUMENT);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(2, 2,
			lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT);

}

void pouring::weight(void)
{
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_weight.trj", 1, lib::ROBOT_IRP6_ON_TRACK);
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_weight.trj", 1, lib::ROBOT_IRP6_POSTUMENT);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(2, 2,
			lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT);

	/*	if(set_next_ecps_state( (int) WEIGHT, 0, "", 1, lib::ROBOT_IRP6_POSTUMENT))
	 {	return true;}
	 run_extended_empty_gen (false, 1, lib::ROBOT_IRP6_POSTUMENT)) {  return true;  }*/

}

void pouring::meet(void)
{
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_meet.trj", 1, lib::ROBOT_IRP6_ON_TRACK);
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_meet.trj", 1, lib::ROBOT_IRP6_POSTUMENT);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(2, 2,
			lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT);

	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_track_meet.trj", 1, lib::ROBOT_IRP6_ON_TRACK);
	run_extended_empty_gen(false, 1, lib::ROBOT_IRP6_ON_TRACK);

	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_pour.trj", 1, lib::ROBOT_IRP6_ON_TRACK);
	run_extended_empty_gen(false, 1, lib::ROBOT_IRP6_ON_TRACK);

}

void pouring::pour(void)
{

	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_POURING, 0, "", 1, lib::ROBOT_IRP6_ON_TRACK);
	run_extended_empty_gen(false, 1, lib::ROBOT_IRP6_ON_TRACK);

	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_pour_2.trj", 1, lib::ROBOT_IRP6_ON_TRACK);
	run_extended_empty_gen(false, 1, lib::ROBOT_IRP6_ON_TRACK);

	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_pour_3.trj", 1, lib::ROBOT_IRP6_ON_TRACK);
	run_extended_empty_gen(false, 1, lib::ROBOT_IRP6_ON_TRACK);

	set_next_ecps_state( (int) ecp_mp::task::ECP_END_POURING, 0, "", 1, lib::ROBOT_IRP6_ON_TRACK);
	run_extended_empty_gen(false, 1, lib::ROBOT_IRP6_ON_TRACK);

	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_pour_4.trj", 1, lib::ROBOT_IRP6_ON_TRACK);
	run_extended_empty_gen(false, 1, lib::ROBOT_IRP6_ON_TRACK);

}

void pouring::go_back(void)
{
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_track_put_back.trj", 1, lib::ROBOT_IRP6_ON_TRACK);
	run_extended_empty_gen(false, 1, lib::ROBOT_IRP6_ON_TRACK);

	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_go_back.trj", 1, lib::ROBOT_IRP6_ON_TRACK);
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_go_back.trj", 1, lib::ROBOT_IRP6_POSTUMENT);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(2, 2,
			lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT);

}

void pouring::put_back(void)
{
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_put_back.trj", 1, lib::ROBOT_IRP6_ON_TRACK);
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_put_back.trj", 1, lib::ROBOT_IRP6_POSTUMENT);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(2, 2,
			lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT);

	//rozwarcie szczek chwytakow
	set_next_ecps_state( (int) ecp_mp::task::LET_GO, 0, "", 1, lib::ROBOT_IRP6_ON_TRACK);

	set_next_ecps_state( (int) ecp_mp::task::LET_GO, 0, "", 1, lib::ROBOT_IRP6_POSTUMENT);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(2, 2,
			lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT);

	//wycofanie chwytakow
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_dep_1.trj", 1, lib::ROBOT_IRP6_ON_TRACK);

	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_dep_1.trj", 1, lib::ROBOT_IRP6_POSTUMENT);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(2, 2,
			lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT);

}

void pouring::depart(void)
{
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_ot_dep_2.trj", 1, lib::ROBOT_IRP6_ON_TRACK);
	set_next_ecps_state( (int) ecp_mp::task::ECP_GEN_SMOOTH, 0, "trj/pouring/irp6_p_dep_2.trj", 1, lib::ROBOT_IRP6_POSTUMENT);

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(2, 2,
			lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT, lib::ROBOT_IRP6_ON_TRACK, lib::ROBOT_IRP6_POSTUMENT);
}

pouring::pouring(lib::configurator &_config) :
	task(_config)
{
	/*
	// Powolanie czujnikow
	sensor_m[lib::SENSOR_FORCE_ON_TRACK]=new ecp_mp_schunk_sensor(lib::SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);

	sensor_m[lib::SENSOR_FORCE_POSTUMENT] = new ecp_mp_schunk_sensor(lib::SENSOR_FORCE_POSTUMENT, "[vsp_force_irp6p]", *this);

	 // Konfiguracja wszystkich czujnikow
	BOOST_FOREACH(ecp_mp::sensor_item_t & sensor_item, sensor_m) {
		sensor_item.second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_item.second->configure_sensor();
	}
	*/
}

task* return_created_mp_task(lib::configurator &_config)
{
	return new pouring(_config);
}

void pouring::main_task_algorithm(void)
{


	approach();
	printf("Approach skonczony\n");
	grab();
	printf("Grab skonczony\n");
	weight();
	printf("Weight skonczony\n");
	meet();
	printf("Meet skonczony\n");
	pour();
	printf("Pour skonczony\n");
	go_back();
	printf("Go back skonczony\n");
	put_back();
	printf("Put back skonczony\n");
	depart();
	printf("Depart skonczony\n");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp

