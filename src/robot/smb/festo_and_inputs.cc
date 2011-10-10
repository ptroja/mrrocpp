/*!
 * \file festo_and_inputs.cc
 * \brief File containing the festo and inputs class methods
 *
 * \author yoyek
 * \date 2011
 *
 */

#include "base/edp/edp_e_motor_driven.h"
#include "const_smb.h"
#include "edp_e_smb.h"
#include "../canopen/gateway_epos_usb.h"
#include "../canopen/gateway_socketcan.h"
#include "../festo/cpv.h"
#include "../maxon/epos.h"
#include "festo_and_inputs.h"

namespace mrrocpp {
namespace edp {
namespace smb {

festo_and_inputs::festo_and_inputs(effector &_master) :
		master(_master)
{

}

festo_and_inputs::~festo_and_inputs()
{

}

bool festo_and_inputs::is_upper_halotron_avtive(int leg_number)
{

	return true;
}

bool festo_and_inputs::is_lower_halotron_avtive(int leg_number)
{
	return true;
}

bool festo_and_inputs::is_attached(int leg_number)
{
	return true;
}

void festo_and_inputs::set_detach(int leg_number, bool value)
{

}

void festo_and_inputs::set_move_up(int leg_number, bool value)
{

}

void festo_and_inputs::set_move_down(int leg_number, bool value)
{

}

void festo_and_inputs::set_clean(int leg_number, bool value)
{

}

void festo_and_inputs::read_state()
{
	epos_inputs = master.epos_di_node->readDInput();
}

void festo_and_inputs::execute_command()
{

}

} // namespace smb
} // namespace edp
} // namespace mrrocpp

