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

festo_and_inputs::festo_and_inputs(effector &_master, boost::shared_ptr <maxon::epos> _epos_di_node, boost::shared_ptr <
		festo::cpv> _cpv10) :
		master(_master), epos_di_node(_epos_di_node), cpv10(_cpv10)
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
	epos_inputs = epos_di_node->readDInput();

	group_one_current_output = cpv10->readOutputs(1);
	std::cout << "group_one_current_output 1= " << group_one_current_output << std::endl;
	group_two_current_output = cpv10->readOutputs(2);
	std::cout << "group_two_current_output 1= " << group_two_current_output << std::endl;
}

void festo_and_inputs::execute_command()
{

}

} // namespace smb
} // namespace edp
} // namespace mrrocpp

