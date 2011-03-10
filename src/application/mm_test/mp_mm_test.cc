
#include <cstdio>
#include <unistd.h>
#include <cstring>
#include <iostream>

#include "mp_mm_test.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"

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
	ACTIVATE_MP_ROBOT(irp6ot_m);

	ACTIVATE_MP_ROBOT(irp6p_m);


}

mmtest::mmtest(lib::configurator &_config) :
	task(_config)
{

}



void mmtest::runWaitFunction(int time)
{
wait_ms(time);
}
/*
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
void mmtest::main_task_algorithm(void)
{

sr_ecp_msg->message("Nowa seria");
set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/swarm_demo/trajectory_postument_joint.trj", 0, 1,
		lib::irp6p_m::ROBOT_NAME.c_str());
set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/swarm_demo/trajectory_track_joint.trj", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
run_extended_empty_gen_and_wait(2, 2, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6p_m::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6p_m::ROBOT_NAME.c_str());

//runWaitFunction(5000);


sr_ecp_msg->message("test end");
}

} // namespace task
} // namespace mp
} // namespace mrrocpp


