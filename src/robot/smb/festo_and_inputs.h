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

	// state of the legs

	lib::smb::ALL_LEGS_VARIANT current_legs_state, next_legs_state;

	std::bitset <16> epos_inputs;
	std::bitset <8> current_output[3], desired_output[3];

	bool robot_test_mode;

public:
	festo_and_inputs(effector &_master);
	~festo_and_inputs();

	void determine_legs_state();

	/*!
	 * \brief festo command variant in move_arm
	 */
	void festo_command();

	/*!
	 * \brief festo command all_down variant in move_arm
	 */
	void festo_command_all_down(lib::smb::festo_command_td& festo_command);

	/*!
	 * \brief moves all legs that are in the upper position down and detach them
	 */

	void move_one_or_two_down();

	/*!
	 * \brief festo command one_up_two_down variant in move_arm
	 */
	void festo_command_one_up_two_down(lib::smb::festo_command_td& festo_command);

	/*!
	 * \brief festo command two_up_one_down variant in move_arm
	 */
	void festo_command_two_up_one_down(lib::smb::festo_command_td& festo_command);

	/*!
	 * \brief festo command two_up_one_down variant in move_arm
	 */
	void festo_command_all_up(lib::smb::festo_command_td& festo_command);

	/*!
	 * \brief festo reply in test_mode
	 * returns true if test_mode is activated
	 */
	bool festo_test_mode_set_reply(lib::smb::festo_command_td& festo_command);

	void initiate();

	bool is_upper_halotron_active(int leg_number);
	bool is_lower_halotron_active(int leg_number);
	bool is_attached(int leg_number);

	void set_detach(int leg_number, bool value);
	void set_move_up(int leg_number, bool value);
	void set_move_down(int leg_number, bool value);
	void set_clean(int leg_number, bool value);

	void read_state();
	void create_reply();
	void execute_command();
};

} // namespace smb
} // namespace edp
} // namespace mrrocpp

#endif

