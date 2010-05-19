/*!
 * \file edp_e_bird_hand.h
 * \brief File containing the declaration of edp::bird_hand::effector class.
 *
 * \author yoyek
 * \date 2009
 *
 */

#ifndef __EDP_E_BIRD_HAND_H
#define __EDP_E_BIRD_HAND_H

#include "edp/common/edp_e_manip.h"
#include "lib/robot_consts/bird_hand_const.h"

namespace mrrocpp {
namespace edp {
namespace bird_hand {

// Klasa reprezentujaca robota IRp-6 na postumencie.
/*!
 * \brief class of EDP SwarmItFix parallel kinematic manipulator
 *
 * It is the base of the head mounted on the mobile base.
 */
class effector: public common::manip_effector {
protected:
	lib::bird_hand_cbuffer ecp_edp_cbuffer;
	lib::bird_hand_rbuffer edp_ecp_rbuffer;

	/*!
	 * \brief method,  creates a list of available kinematic models for bird_hand effector.
	 *
	 * Here it is parallel manipulator direct and inverse kinematic transform
	 * and motor to joint transform
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
	 * \brief method to move robot arm
	 *
	 * it chooses the single thread variant from the manip_effector
	 */
	void move_arm(const lib::c_buffer &instruction); // przemieszczenie ramienia

	void get_controller_state(lib::c_buffer &instruction);

	/*!
	 * \brief method to get position of the arm
	 *
	 * Here it calls common::manip_effector::get_arm_position_get_arm_type_switch
	 */
	void get_arm_position(bool read_hardware, lib::c_buffer &instruction); // odczytanie pozycji ramienia

	/*!
	 * \brief method to set the robot model commanded by ECP
	 *
	 * The model consists of tool_frame and models handled in set_robot_model method of motor_driven_effector called here
	 */
	void set_robot_model(const lib::c_buffer &instruction);

	/*!
	 * \brief method to get (read) the robot model
	 *
	 * The model consists of tool_frame and models handled in set_robot_model method of motor_driven_effector called here.
	 * Then it is sent to the ECP.
	 */
	void get_robot_model(lib::c_buffer &instruction);

	/*!
	 * \brief method to choose master_order variant
	 *
	 * IHere the single thread variant is chosen
	 */
	void master_order(common::MT_ORDER nm_task, int nm_tryb);

	/*!
	 * \brief method to receive instruction from ECP
	 *
	 * IT also makes initial ECP command interpretation..
	 */
	lib::INSTRUCTION_TYPE receive_instruction(void); // by YW

	/*!
	 * \brief method to reply to ECP
	 *
	 * Basing on the previous computation..
	 */
	void reply_to_instruction(void);

};

} // namespace bird_hand
} // namespace edp
} // namespace mrrocpp


#endif
