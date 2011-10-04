/*!
 * \file edp_e_smb.h
 * \brief File containing the declaration of edp::smb::effector class.
 *
 * \author yoyek
 * \date 2009
 *
 */

#ifndef __FESTO_AND_INPUTS_H
#define __FESTO_AND_INPUTS_H

#include "base/edp/edp_e_motor_driven.h"
#include "const_smb.h"

#include "../canopen/gateway_epos_usb.h"
#include "../canopen/gateway_socketcan.h"
#include "../festo/cpv.h"
#include "../maxon/epos.h"

namespace mrrocpp {
namespace edp {
namespace smb {

class effector;

class festo_and_inputs
{
private:
	effector &master;

public:
	festo_and_inputs(effector &_master);
	~festo_and_inputs();

	bool is_upper_halotron_avtive(int leg_number);
	bool is_lower_halotron_avtive(int leg_number);
	bool is_attached(int leg_number);

	void set_detach(int leg_number, bool value);
	void set_move_up(int leg_number, bool value);
	void set_move_down(int leg_number, bool value);
	void set_clean(int leg_number, bool value);

	void read_state();
	void execute_command();
};

} // namespace smb
} // namespace edp
} // namespace mrrocpp

#endif

