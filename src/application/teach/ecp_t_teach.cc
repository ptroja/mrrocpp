// ------------------------------------------------------------------------
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------

#include <cstdio>
#include <cstring>

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

#include "application/teach/ecp_t_teach.h"
#include "generator/ecp/teach_in/ecp_g_teach_in.h"

#include "base/ecp/ecp_robot.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
teach::teach(lib::configurator &_config) :
		common::task::task(_config)
{
	if (config.robot_name == lib::irp6ot_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6ot_m::robot(*this);
	} else if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_m::robot(*this);
	} else {
		fprintf(stderr, "unknown robot \"%s\" in teach task\n", config.section_name.c_str());
		BOOST_THROW_EXCEPTION(exception::fe_r());
	}

	tig = new generator::teach_in(*this);

	sr_ecp_msg->message("ecp loaded");
}

void teach::main_task_algorithm(void)
{

	if (ecp_m_robot->robot_name == lib::irp6ot_m::ROBOT_NAME) {
		sr_ecp_msg->message("ecp teach irp6ot_m");
	} else if (ecp_m_robot->robot_name == lib::irp6p_m::ROBOT_NAME) {
		sr_ecp_msg->message("ecp teach irp6p_m");
	}

	if (operator_reaction("Teach in? ")) {
		tig->flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
		tig->teach(lib::ECP_MOTOR, "Teach-in the trajectory\n");
	}

	if (operator_reaction("Save trajectory? ")) {
		tig->save_file(lib::ECP_MOTOR);
	}

	if (operator_reaction("Load trajectory? ")) {
		tig->load_file_from_ui();
	}

	// Aktualnie petla wykonuje sie jednokrotnie, gdyby MP przejal sterowanie
	// to petle mozna przerwac przez STOP lub przez polecenie lib::END_MOTION wydane
	// przez MP
	//  printf("w ecp for\n");
	tig->Move();
	// 	 printf("w ecp for za move\n");
	// Oczekiwanie na STOP
	termination_notice();
}

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new teach(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

