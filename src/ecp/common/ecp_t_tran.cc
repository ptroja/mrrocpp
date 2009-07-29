// ------------------------------------------------------------------------
//   ecp_t_tran.cc - przezroczyste wersja dla dowolnego z robotow
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------


#include <stdio.h>
#include <string.h>

#include "lib/srlib.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/irp6_mechatronika/ecp_r_irp6m.h"
#include "ecp/conveyor/ecp_r_conv.h"
#include "ecp/speaker/ecp_r_speaker.h"
#include "ecp/polycrank/ecp_r_polycrank.h"

#include "ecp/common/ecp_t_tran.h"
#include "ecp/common/ecp_generator_t.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {


// KONSTRUKTORY
tran::tran(lib::configurator &_config) :
	task(_config)
	{}

// methods for ECP template to redefine in concrete classes
void tran::task_initialization(void)
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	if (!strcmp(config.section_name, "[ecp_irp6_on_track]"))
	{
		ecp_m_robot = new irp6ot::robot (*this);
	}
	else if (!strcmp(config.section_name, "[ecp_irp6_postument]"))
	{
		ecp_m_robot = new irp6p::robot (*this);
	}
	else if (!strcmp(config.section_name, "[ecp_conveyor]"))
	{
		ecp_m_robot = new conveyor::robot (*this);
	}
	else if (!strcmp(config.section_name, "[ecp_speaker]"))
	{
		ecp_m_robot = new speaker::robot (*this);
	}
	else if (!strcmp(config.section_name, "[ecp_irp6_mechatronika]"))
	{
		ecp_m_robot = new irp6m::robot (*this);
	}
	else if (!strcmp(config.section_name, "[ecp_polycrank]"))
	{
		ecp_m_robot = new polycrank::robot (*this);
	}

	sr_ecp_msg->message("ECP loaded");
}


void tran::main_task_algorithm(void)
{
	generator::transparent gt (*this);
	sr_ecp_msg->message("Ruch");

	gt.Move();

}

task* return_created_ecp_task (lib::configurator &_config)
{
	return new tran(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
