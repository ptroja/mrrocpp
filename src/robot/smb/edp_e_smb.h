/*!
 * \file edp_e_smb.h
 * \brief File containing the declaration of edp::smb::effector class.
 *
 * \author yoyek
 * \date 2011
 *
 */

#ifndef __EDP_E_SMB_H
#define __EDP_E_SMB_H

#include "base/edp/edp_e_motor_driven.h"
#include "const_smb.h"
#include "festo_and_inputs.h"
#include "../canopen/gateway_epos_usb.h"
#include "../canopen/gateway_socketcan.h"
#include "../festo/cpv.h"
#include "../maxon/epos.h"

namespace mrrocpp {
namespace edp {
namespace smb {

class festo_and_inputs;

/*!
 * \brief class of EDP SwarmItFix mobile base
 *
 * This mobile platform is the base of the SPKM manipulator
 */
class effector : public common::motor_driven_effector
{

	friend class festo_and_inputs;

protected:

	const static int FESTO_ADRESS = 10;

	//! Access to the CAN gateway unit
	boost::shared_ptr <canopen::gateway> gateway;

	//! PKM axes.
	boost::shared_ptr <maxon::epos> axisA, axisB, axisC, axis1, axis2, axis3;

	//! Names of PKM axes.
	boost::array <std::string, mrrocpp::lib::smb::NUM_OF_SERVOS> axesNames;

	//! Axes container.
	boost::array <maxon::epos *, mrrocpp::lib::smb::NUM_OF_SERVOS> axes;

	//! Digitial_input axis
	boost::shared_ptr <maxon::epos> legs_rotation_node;

	//! Axis responsible for rotation of the PKM (upper SMB joint).
	boost::shared_ptr <maxon::epos> pkm_rotation_node;

	//! festo shared ptr
	boost::shared_ptr <festo::cpv> cpv10;

	/*!
	 * \brief Variable storing the relative zero position of the motor rotating legs.
	 * Set when all legs are out.
	 */
	int32_t legs_relative_zero_position;

	/*!
	 * \brief Variable storing the voltage adequate for zero position (in milivolts).
	 */
	int32_t pkm_zero_position_voltage;

	/*!
	 * \brief Variable storing synchronization offset (offset from *voltage* zero position).
	 */
	int32_t pkm_zero_position_offset;

	//! Default axis velocity [rpm]
	static const uint32_t Vdefault[mrrocpp::lib::smb::NUM_OF_SERVOS];

	//! Default axis acceleration [rpm/s]
	static const uint32_t Adefault[mrrocpp::lib::smb::NUM_OF_SERVOS];

	//! Default axis deceleration [rpm/s]
	static const uint32_t Ddefault[mrrocpp::lib::smb::NUM_OF_SERVOS];

	/*!
	 * \brief Method responsible for parsing of the command for motors controlling the legs and SPKM rotation.
	 * \author tkornuta
	 */
	void parse_motor_command();

	/*!
	 * \brief Method responsible for motion of motors controlling the legs and SPKM rotation.
	 * \author tkornuta
	 */
	void execute_motor_motion();

	/*!
	 * \brief pointer to festo_and_inputs class
	 */
	festo_and_inputs* fai;

	//! Method checks the state of EPOS controllers.
	void check_controller_state();

	/*!
	 * \brief method,  creates a list of available kinematic models for smb effector.
	 *
	 * Here it is  motor to joint transform of two legs and manipulator base rotation motor
	 */
	virtual void create_kinematic_models_for_given_robot(void);

public:

	/*!
	 * \brief class constructor
	 *
	 * The attributes are initialized here.
	 */
	effector(common::shell &_shell, lib::robot_name_t l_robot_name);

	/*!
	 * \brief method to create threads other then EDP master thread.
	 *
	 * Here there is only one extra thread - reader_thread.
	 */
	void create_threads();

	/*!
	 * @brief Motors synchronization - utilizes velocity motion based on the reading of potentiometer.
	 *
	 * This method synchronizes motor rotating the SMB.
	 */
	void synchronise();

	/*!
	 * \brief returns current legs state from festo_and_inputs class
	 *
	 */
	lib::smb::ALL_LEGS_VARIANT current_legs_state(void);

	/*!
	 * \brief returns next (desired) legs state from festo_and_inputs class
	 *
	 */
	lib::smb::ALL_LEGS_VARIANT next_legs_state(void);

	/*!
	 * \brief Executes the *move_arm* command.
	 *
	 * It chooses the single thread variant from the motor_driven_effector.
	 *
	 * \param [in] instruction_ - Received command. Parameter UNUSED! due to the fact, that this is a single threaded driver.
	 */
	void move_arm(const lib::c_buffer &instruction_);

	/*
	 * \brief Initializes the controller.
	 * Called only once after process creation.
	 *
	 * \param [in] instruction_ - Received command. Parameter UNUSED! due to the fact, that this is a single threaded driver.
	 */
	void get_arm_position(bool read_hardware, lib::c_buffer &instruction_);

	/*!
	 * \brief Method initializes SMB variables (including motors, joints and frames), depending on working mode (robot_test_mode) and robot state.
	 * Called only once after process creation.
	 *
	 * \param [in] instruction_ - Received command. Parameter UNUSED! due to the fact, that this is a single threaded driver.
	 */
	void get_controller_state(lib::c_buffer &instruction_);

	/*!
	 * \brief method to choose master_order variant
	 *
	 * IHere the single thread variant is chosen
	 */
	void master_order(common::MT_ORDER nm_task, int nm_tryb);

	/*!
	 * \brief method to receive instruction from ecp of particular type
	 */
	lib::INSTRUCTION_TYPE receive_instruction();

	/*!
	 * \brief method to reply to ecp with class of particular type
	 */
	void variant_reply_to_instruction();

	/*!
	 * \brief The particular type of instruction send form ECP to EDP
	 */
	lib::smb::c_buffer instruction;

	/*!
	 * \brief The particular type of reply send form EDP to ECP
	 */
	lib::smb::r_buffer reply;

	/*!
	 * \brief bool value if the cleaning is activated
	 *
	 * It is taken from configuration data.
	 */
	bool cleaning_active;

};

} // namespace smb
} // namespace edp
} // namespace mrrocpp

#endif
