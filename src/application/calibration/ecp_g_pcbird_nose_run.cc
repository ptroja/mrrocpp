#include "base/ecp/ecp_robot.h"
#include "ecp_g_pcbird_nose_run.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////
//
// pcbird_nose_run generator
//
//////////////////////////////////////////////////////////////////////////////////////////////////

pcbird_nose_run::pcbird_nose_run(common::task::task& _ecp_task, int step) :
	tff_nose_run(_ecp_task, step)
{
	count = 0;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool pcbird_nose_run::first_step()
{
	//bedziemy pobierali dane o robocie
	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;

	return tff_nose_run::first_step();
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step ---------------------------------------
// ----------------------------------------------------------------------------------------------

bool pcbird_nose_run::next_step()
{
	++count;

	//co jakis czas generator sie zatrzymuje
	if (count > 10) {
		count = 0;
		return false;
	}

	return tff_nose_run::next_step();
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


