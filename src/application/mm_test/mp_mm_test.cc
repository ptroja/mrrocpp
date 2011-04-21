
#include <cstdio>
#include <unistd.h>
#include <cstring>
#include <iostream>

#include "mp_mm_test.h"
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

#include "ecp_mp_g_g_mm_test.h"


namespace mrrocpp {
namespace mp {
namespace task {
task* return_created_mp_task(lib::configurator &_config)
{
	return new mmtest(_config);
}



// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void mmtest::create_robots()
{
	//ACTIVATE_MP_ROBOT(irp6ot_m);

	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(irp6p_tfg);


}

mmtest::mmtest(lib::configurator &_config) :
		task(_config)
{

}


/*
void mmtest::runWaitFunction(int time)
{
wait_ms(time);
}

void mmtest::runEmptyGen()
{
run_extended_empty_gen_base(state.getNumArgument(), 1, (state.getRobot()).c_str());
}

void mmtest::runEmptyGenForSet(common::State &state)
{
run_extended_empty_gen_and_wait(
		state.robotSet->firstSetCount, state.robotSet->secondSetCount, state.robotSet->firstSet,
		state.robotSet->secondSet);
}

void mmtest::executeMotion(common::State &state)
{

int trjConf = config.value<int>("trajectory_from_xml", "[xml_settings]");
if (trjConf && state.getGeneratorType() == ecp_mp::generator::ECP_GEN_NEWSMOOTH) {
	set_next_ecps_state(state.getGeneratorType(), state.getNumArgument(), state.getStateID(), 0, 1,
			(state.getRobot()).c_str());
} else {
	set_next_ecps_state(state.getGeneratorType(), state.getNumArgument(), state.getStringArgument(), 0, 1,
			(state.getRobot()).c_str());
}
}
*/
void mmtest::move_down(double mm)
{
	sr_ecp_msg->message("test0");
char str[10];
sprintf (str, "1 %lf", mm);
sr_ecp_msg->message("test1");
	set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, str, 0, 1,
			lib::irp6p_m::ROBOT_NAME.c_str());
	sr_ecp_msg->message("test2");

}
void mmtest::move_right(double mm)
{
	sr_ecp_msg->message("test0");
char str[10];
sprintf (str, "2 %lf", mm);
sr_ecp_msg->message("test1");
	set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, str, 0, 1,
			lib::irp6p_m::ROBOT_NAME.c_str());
	sr_ecp_msg->message("test2");

}
void mmtest::move_back(double mm)
{
	sr_ecp_msg->message("test0");
char str[10];
sprintf (str, "3 %lf", mm);
sr_ecp_msg->message("test1");
	set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, str, 0, 1,
			lib::irp6p_m::ROBOT_NAME.c_str());
	sr_ecp_msg->message("test2");

}

void mmtest::main_task_algorithm(void)
{
	sr_ecp_msg->message("mp start");
/*
std::vector<Point> path;
Point p;
p.x=5;
p.y=5;
path.push_back(p);

p.x++;
path.push_back(p);

p.y++;
path.push_back(p);

p.x++;
path.push_back(p);

p.y--;
path.push_back(p);

p.y--;
path.push_back(p);

p.x++;
path.push_back(p);

p.x++;
path.push_back(p);

p.y--;
path.push_back(p);

p.x--;
path.push_back(p);
*/

/*
sr_ecp_msg->message("New edge_follow_mr series");

std::stringstream ss(std::stringstream::in | std::stringstream::out);

lib::Xyz_Euler_Zyz_vector rel_eu(0, 0, 0, -1.57, 1.57, 1.57);
lib::Homog_matrix tmp_hm(rel_eu);
lib::Xyz_Angle_Axis_vector rel_aa;
tmp_hm.get_xyz_angle_axis(rel_aa);

ss << rel_aa;

sr_ecp_msg->message(ss.str().c_str());
*/
//set_next_ecps_state(ecp_mp::sub_task::ECP_ST_TFF_NOSE_RUN, (int) 5, "", 0, 1, manipulator_name.c_str());
//run_extended_empty_gen_and_wait(1, 1, manipulator_name.c_str(), manipulator_name.c_str());

//set_next_ecps_state(ecp_mp::sub_task::EDGE_FOLLOW, (int) 5, "", 0, 1, manipulator_name.c_str());
//run_extended_empty_gen_and_wait(1, 1, manipulator_name.c_str(), manipulator_name.c_str());


lib::robot_name_t manipulator_name;
lib::robot_name_t gripper_name;

manipulator_name = lib::irp6p_m::ROBOT_NAME;
gripper_name = lib::irp6p_tfg::ROBOT_NAME;


sr_ecp_msg->message("SZCZEKI WYSZCZERZ");
set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/mm_test/szczeki2.trj", 0, 1,
	lib::irp6p_m::ROBOT_NAME.c_str());
run_extended_empty_gen_and_wait(1, 1, lib::irp6p_m::ROBOT_NAME.c_str(),lib::irp6p_m::ROBOT_NAME.c_str());


char tmp_string[lib::MP_2_ECP_NEXT_STATE_STRING_SIZE];
lib::irp6_tfg::mp_to_ecp_parameters mp_ecp_command;

mp_ecp_command.desired_position = 0.077;
memcpy(tmp_string, &mp_ecp_command, sizeof(mp_ecp_command));

sr_ecp_msg->message("OTWORZ");
set_next_ecps_state(ecp_mp::generator::ECP_GEN_TFG, (int) 5, tmp_string, sizeof(mp_ecp_command), 1, gripper_name.c_str());
run_extended_empty_gen_and_wait(1, 1, gripper_name.c_str(), gripper_name.c_str());

wait_ms(1000);

mp_ecp_command.desired_position = 0.0715;
memcpy(tmp_string, &mp_ecp_command, sizeof(mp_ecp_command));

sr_ecp_msg->message("ZAMKNIJ");
set_next_ecps_state(ecp_mp::generator::ECP_GEN_TFG, (int) 5, tmp_string, sizeof(mp_ecp_command), 1, gripper_name.c_str());
run_extended_empty_gen_and_wait(1, 1, gripper_name.c_str(), gripper_name.c_str());

wait_ms(3000);


sr_ecp_msg->message("SZCZEKI W DOL");
set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/mm_test/poz_pocz.trj", 0, 1,
		lib::irp6p_m::ROBOT_NAME.c_str());
run_extended_empty_gen_and_wait(1, 1, lib::irp6p_m::ROBOT_NAME.c_str(),lib::irp6p_m::ROBOT_NAME.c_str());



sr_ecp_msg->message("BIAS");
set_next_ecps_state(ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE, (int) 5, "", 0, 1, manipulator_name.c_str());
run_extended_empty_gen_and_wait(1, 1, manipulator_name.c_str(), manipulator_name.c_str());



sr_ecp_msg->message("MOJ GENERATOR POPYCHAJACY");
set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_MM_TEST, (int) 5, "", 0, 1,
	lib::irp6p_m::ROBOT_NAME.c_str());
run_extended_empty_gen_and_wait(1, 1, lib::irp6p_m::ROBOT_NAME.c_str(),lib::irp6p_m::ROBOT_NAME.c_str());

/*

move_right(-0.15);
run_extended_empty_gen_and_wait(1, 1, lib::irp6p_m::ROBOT_NAME.c_str(),lib::irp6p_m::ROBOT_NAME.c_str());
*/

//move_down(0.2);
//run_extended_empty_gen_and_wait(1, 1, lib::irp6p_m::ROBOT_NAME.c_str(),lib::irp6p_m::ROBOT_NAME.c_str());
/*
double n=0.05;//szerokosc pola-5cm
std::vector<Point>::reverse_iterator rit,prit;

for ( prit=rit=path.rbegin(),rit++ ; rit < path.rend(); ++rit, ++prit )
{
	if((*rit).x<(*prit).x)
	{
		move_right(-n);
		run_extended_empty_gen_and_wait(1, 1, lib::irp6p_m::ROBOT_NAME.c_str(),lib::irp6p_m::ROBOT_NAME.c_str());
	}
	if((*rit).x>(*prit).x)
	{
		move_right(n);
		run_extended_empty_gen_and_wait(1, 1, lib::irp6p_m::ROBOT_NAME.c_str(),lib::irp6p_m::ROBOT_NAME.c_str());
	}
	if((*rit).y<(*prit).y)
	{
		move_back(n);
		run_extended_empty_gen_and_wait(1, 1, lib::irp6p_m::ROBOT_NAME.c_str(),lib::irp6p_m::ROBOT_NAME.c_str());
	}
	if((*rit).y>(*prit).y)
	{
		move_back(-n);
		run_extended_empty_gen_and_wait(1, 1, lib::irp6p_m::ROBOT_NAME.c_str(),lib::irp6p_m::ROBOT_NAME.c_str());
	}
}
*/
sr_ecp_msg->message("mp end");
}

} // namespace task
} // namespace mp
} // namespace mrrocpp


