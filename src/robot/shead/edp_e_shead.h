/*!
 * \file edp_e_shead.h
 * \brief File containing the declaration of edp::shead::effector class.
 *
 * \author yoyek
 * \date 2009
 *
 */

#ifndef __EDP_E_SHEAD_H
#define __EDP_E_SHEAD_H

#include "base/edp/edp_e_motor_driven.h"
#include "dp_shead.h"

#include "robot/canopen/gateway.h"
#include "robot/maxon/epos.h"

namespace mrrocpp {
namespace edp {
namespace shead {

// Klasa reprezentujaca robota IRp-6 na postumencie.

/*!
 * \brief class of EDP SwarmItFix head effector
 *
 * This head is built on top of the SPKM manipulator
 */
class effector : public common::motor_driven_effector
{
protected:
	//! Access to the CAN gateway unit
	boost::shared_ptr <canopen::gateway> gateway;

	//! Digitial_input axis
	boost::shared_ptr <maxon::epos> epos_node;

	//! Default axis velocity [rpm]
	static const uint32_t Vdefault;

	//! Default axis acceleration [rpm/s]
	static const uint32_t Adefault;

	//! Default axis deceleration [rpm/s]
	static const uint32_t Ddefault;

	//! Parse command for motors.
	void parse_motor_command();

	//! Execute parsed command.
	void execute_motor_motion();

	//! Check state of the EPOS controller.
	void check_controller_state();

	lib::shead::cbuffer ecp_edp_cbuffer;
	lib::shead::rbuffer edp_ecp_rbuffer;

	//! Virtual state for test mode
	struct _virtual_state {
		//! Solidification state
		lib::shead::solidification_state_t solidification_state;

		//! Vacuum state
		lib::shead::vacuum_state_t vacuum_state;

		//! Setup startup values
		_virtual_state() :
			solidification_state(lib::shead::SOLIDIFICATION_STATE_OFF),
			vacuum_state(lib::shead::VACUUM_STATE_OFF)
		{};
	} virtual_state;

	// Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
	/*!
	 * \brief method,  creates a list of available kinematic models for shead effector.
	 *
	 * It will be used if any motor will be commanded to move. Then motor to joint transform will be implemented in kinematics.
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
	 * @brief motors synchronization
	 *
	 * This method synchronizes motors of the robots.
	 */
	void synchronise();

	void get_controller_state(lib::c_buffer &instruction);

	/*!
	 * \brief method to set position of the motors or joints
	 *
	 * It will be used if there will be any motor used.
	 */
	void move_arm(const lib::c_buffer &instruction); // przemieszczenie ramienia

	/*!
	 * \brief method to get position of the motors or joints
	 *
	 * It will be used if there will be any motor used.
	 */

	void get_arm_position(bool read_hardware, lib::c_buffer &instruction); // odczytanie pozycji ramienia

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
