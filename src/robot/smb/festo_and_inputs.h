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
#include <bitset>

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

	friend class effector;

private:
	effector &master;

	boost::shared_ptr <maxon::epos> epos_di_node;

	//! festo shared ptr
	boost::shared_ptr <festo::cpv> cpv10;

	std::bitset <16> epos_inputs;
	std::bitset <8> group_one_current_output, group_two_current_output, group_one_desired_output,
			group_two_desired_output;

public:
	festo_and_inputs(effector &_master, boost::shared_ptr <maxon::epos> _epos_di_node, boost::shared_ptr <festo::cpv> _cpv10);
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

