/*!
 * \brief File containing the declaration of the festo and inputs class methods
 *
 * \author yoyek
 * \date 2011
 *
 */

#ifndef __FESTO_AND_INPUTS_H
#define __FESTO_AND_INPUTS_H

#include <bitset>

#include "dp_smb.h"
#include "base/edp/edp_e_motor_driven.h"
#include "../canopen/gateway_epos_usb.h"
#include "../canopen/gateway_socketcan.h"
#include "../festo/cpv.h"
#include "../maxon/epos.h"

#define FAI_SINGLE_DELAY 20
#define FAI_DELAY_MAX_ITERATION 250

#define EPOS_L1_HAL_UP 11
#define EPOS_L3_HAL_UP 13
#define EPOS_L2_HAL_UP 15
#define EPOS_L1_HAL_DOWN 10
#define EPOS_L3_HAL_DOWN 12
#define EPOS_L2_HAL_DOWN 14

namespace mrrocpp {
namespace edp {
namespace smb {

class effector;

class festo_and_inputs
{

	friend class effector;

private:

	/*!
	 * \brief reference to master object of effector class
	 */
	effector &master;

	/*!
	 * \brief the array is used to memorize current state of the leg in context of current operation
	 */
	bool checked[lib::smb::LEG_CLAMP_NUMBER];

	/*!
	 * \brief shared pointer to epos controller interface needed to read the input data of current legs state
	 */
	boost::shared_ptr <maxon::epos> epos_di_node;

	/*!
	 * \brief shared pointer to festo interface to control pneumatic outputs
	 */
	boost::shared_ptr <festo::cpv> cpv10;

	/*!
	 * \brief state of the legs i.e. number of legs in upper and lower position
	 */
	lib::smb::ALL_LEGS_VARIANT current_legs_state, next_legs_state;

	/*!
	 * \brief current input data from epos interface
	 */
	std::bitset <16> epos_inputs;

	/*!
	 * \brief current and desired output data of festo controller
	 */
	std::bitset <8> current_output[3], desired_output[3];

	/*!
	 * \brief robot_test_mode taken from effector class
	 * it does not change during task executon
	 */
	bool robot_test_mode;

	/*!
	 * \brief current festo command
	 */
	lib::smb::festo_command_td festo_command;

public:

	/*!
	 * \brief festo_and_inputs constructor
	 */
	festo_and_inputs(effector &_master);

	/*!
	 * \brief festo_and_inputs destructor
	 */
	~festo_and_inputs();

	/*!
	 * \brief checks the current legs state
	 */
	void determine_legs_state();

	/*!
	 * \brief determine if the particular leg was checked
	 */
	bool is_checked(int leg_number);

	/*!
	 * \brief set the particular leg checked
	 */
	void set_checked(int leg_number);

	/*!
	 * \brief set the particular leg unchecked
	 */
	void set_unchecked(int leg_number);

	/*!
	 * \brief uncheck all legs
	 */
	void set_all_legs_unchecked();

	/*!
	 * \brief festo command variant in move_arm
	 */
	void command();

	/*!
	 * \brief festo command all_down variant in move_arm
	 */
	void command_all_out();

	/*!
	 * \brief moves all legs that are in the upper position down and detach them
	 */

	void move_one_or_two_out();

	/*!
	 * \brief festo command one_up_two_down variant in move_arm
	 */
	void command_one_in_two_out();

	/*!
	 * \brief festo command two_up_one_down variant in move_arm
	 */
	void command_two_in_one_out();

	/*!
	 * \brief festo command command_all_in variant in move_arm
	 */
	void command_all_in();

	/*!
	 * \brief festo reply in test_mode
	 * returns true if test_mode is activated
	 */
	bool test_mode_set_reply();

	/*!
	 * \brief checks if upper halotron of particular leg is active
	 * \param leg_number counter from 1
	 */
	bool is_inper_halotron_active(int leg_number);

	/*!
	 * \brief checks if lower halotron of particular leg is active
	 * \param leg_number counter from 1
	 */
	bool is_lower_halotron_active(int leg_number);

	/*!
	 * \brief checks if particular leg is attached
	 * \param leg_number counter from 1
	 */
	bool is_attached(int leg_number);

	/*!
	 * \brief set the command to detach particular leg
	 * \param leg_number counter from 1
	 * \param value true or false
	 */
	void set_detach(int leg_number, bool value);

	/*!
	 * \brief set the command to move up particular leg
	 * \param leg_number counter from 1
	 * \param value true or false
	 */
	void set_move_in(int leg_number, bool value);

	/*!
	 * \brief set the command to move down particular leg
	 * \param leg_number counter from 1
	 * \param value true or false
	 */
	void set_move_out(int leg_number, bool value);

	/*!
	 * \brief set the command to clean particular leg
	 * \param leg_number counter from 1
	 * \param value true or false
	 */
	void set_clean(int leg_number, bool value);

	/*!
	 * \brief reads the input state
	 */
	void read_state();

	/*!
	 * \brief prepers the reply buffer for effector class
	 */
	void create_reply();

	/*!
	 * \brief executes the desired command
	 * it communicates with hardware
	 */
	void execute_command();
};

} // namespace smb
} // namespace edp
} // namespace mrrocpp

#endif

