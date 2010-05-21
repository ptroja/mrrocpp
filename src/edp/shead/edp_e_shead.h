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

#include "edp/common/edp_e_motor_driven.h"
#include "lib/robot_consts/shead_const.h"

namespace mrrocpp {
namespace edp {
namespace shead {

// Klasa reprezentujaca robota IRp-6 na postumencie.


/*!
 * \brief class of EDP SwarmItFix head effector
 *
 * This head is built on top of the SPKM manipulator
 */
class effector: public common::motor_driven_effector {
protected:

	lib::shead_cbuffer ecp_edp_cbuffer;
	lib::shead_rbuffer edp_ecp_rbuffer;

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
	effector(lib::configurator &_config);

	/*!
	 * \brief method to create threads other then EDP master thread.
	 *
	 * Here there is only one extra thread - reader_thread.
	 */
	void create_threads();

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
