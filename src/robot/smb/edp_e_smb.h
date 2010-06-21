/*!
 * \file edp_e_smb.h
 * \brief File containing the declaration of edp::smb::effector class.
 *
 * \author yoyek
 * \date 2009
 *
 */

#ifndef __EDP_E_SMB_H
#define __EDP_E_SMB_H

#include "edp/common/edp_e_motor_driven.h"
#include "lib/robot_consts/smb_const.h"

namespace mrrocpp {
namespace edp {
namespace smb {

/*!
 * \brief class of EDP SwarmItFix mobile base
 *
 * This mobile platform is the base of the SPKM manipulator
 */
class effector: public common::motor_driven_effector {
protected:

	lib::smb_cbuffer ecp_edp_cbuffer;
	lib::smb_rbuffer edp_ecp_rbuffer;

	// Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.

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
	effector(lib::configurator &_config);

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

	/*!
	 * \brief method to get position of the motors or joints
	 *
	 * Here it calls common::motor_driven_effector::get_arm_position_get_arm_type_switch
	 */
	void get_arm_position(bool read_hardware, lib::c_buffer &instruction); // odczytanie pozycji ramienia

	void get_controller_state(lib::c_buffer &instruction);

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
