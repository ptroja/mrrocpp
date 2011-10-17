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

private:
	//! Access to the CAN gateway unit
	boost::shared_ptr <canopen::gateway> gateway;

	//! PKM axes.
	boost::shared_ptr <maxon::epos> axisA, axisB, axisC, axis1, axis2, axis3;

	//! Names of PKM axes.
	boost::array <std::string, mrrocpp::lib::smb::NUM_OF_SERVOS> axesNames;

	//! Axes container.
	boost::array <maxon::epos *, mrrocpp::lib::smb::NUM_OF_SERVOS> axes;

	//! Digitial_input axis
	boost::shared_ptr <maxon::epos> epos_di_node;

	//! Axis responsible for rotation of the PKM (upper SMB joint).
	boost::shared_ptr <maxon::epos> pkm_rotation_node;

	//! festo shared ptr
	boost::shared_ptr <festo::cpv> cpv10;

	// state of the legs rotation
	bool is_base_positioned_to_move_legs;

	//! Default axis velocity [rpm]
	static const uint32_t Vdefault[mrrocpp::lib::smb::NUM_OF_SERVOS];

	//! Default axis acceleration [rpm/s]
	static const uint32_t Adefault[mrrocpp::lib::smb::NUM_OF_SERVOS];

	//! Default axis deceleration [rpm/s]
	static const uint32_t Ddefault[mrrocpp::lib::smb::NUM_OF_SERVOS];

	/*!
	 * \brief Method responsible for control of the rotational motors (motor controlling the legs and SPKM rotation).
	 * \author tkornuta
	 */
	void rotational_motors_command();

	/*!
	 * \brief pointer to festo_and_inputs class
	 */
	festo_and_inputs* fai;

protected:

	lib::smb::cbuffer ecp_edp_cbuffer;
	lib::smb::rbuffer edp_ecp_rbuffer;

	// Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.

	/*!
	 * \brief method,  creates a list of available kinematic models for smb effector.
	 *
	 * Here it is  motor to joint transform of two legs and manipulator base rotation motor
	 */
	virtual void create_kinematic_models_for_given_robot(void);

public:

	/*!
	 * @brief Method sets initial values of motor and joint positions.
	 * @note The number_of_servos should be previously set.
	 */
	void reset_variables();

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
	 * \brief method to move robot motors
	 *
	 * it chooses the single thread variant from the motor_driven_effector
	 */
	void move_arm(const lib::c_buffer &instruction); // przemieszczenie ramienia

	lib::smb::ALL_LEGS_VARIANT current_legs_state(void);
	lib::smb::ALL_LEGS_VARIANT next_legs_state(void);

	/*!
	 * \brief method to get position of the motors or joints
	 *
	 * Here it calls common::motor_driven_effector::get_arm_position_get_arm_type_switch
	 */
	void get_arm_position(bool read_hardware, lib::c_buffer &instruction); // odczytanie pozycji ramienia

	void get_controller_state(lib::c_buffer &instruction);

	/*!
	 * @brief motors synchronization
	 *
	 * This method synchronizes motors of the robots.
	 */
	void synchronise();

	/*!
	 * \brief method to choose master_order variant
	 *
	 * IHere the single thread variant is chosen
	 */
	void master_order(common::MT_ORDER nm_task, int nm_tryb);

	/*!
	 * \brief method to deserialize part of the reply
	 *
	 * Currently simple memcpy implementation
	 */
	void instruction_deserialization();

	/*!
	 * \brief method to serialize part of the reply
	 *
	 * Currently simple memcpy implementation
	 */
	void reply_serialization();

};

} // namespace smb
} // namespace edp
} // namespace mrrocpp

#endif
