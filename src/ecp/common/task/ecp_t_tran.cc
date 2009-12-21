// ------------------------------------------------------------------------
//   task/ecp_t_tran.cc - przezroczyste wersja dla dowolnego z robotow
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

#include "ecp/common/task/ecp_t_tran.h"
#include "ecp/common/generator/ecp_g_transparent.h"


namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {


// KONSTRUKTORY
tran::tran(lib::configurator &_config) :
	task(_config)
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	if (config.section_name ==ECP_IRP6_ON_TRACK_SECTION)
	{
		ecp_m_robot = new irp6ot::robot (*this);
	}
	else if (config.section_name == ECP_IRP6_POSTUMENT_SECTION)
	{
		ecp_m_robot = new irp6p::robot (*this);
	}
	else if (config.section_name == ECP_CONVEYOR_SECTION)
	{
		ecp_m_robot = new conveyor::robot (*this);
	}
	else if (config.section_name == ECP_SPEAKER_SECTION)
	{
		ecp_m_robot = new speaker::robot (*this);
	}
	else if (config.section_name ==ECP_IRP6_MECHATRONIKA_SECTION)
	{
		ecp_m_robot = new irp6m::robot (*this);
	}
	else if (config.section_name == ECP_POLYCRACNK_SECTION)
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
