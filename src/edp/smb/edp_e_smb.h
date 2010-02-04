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
#include "lib/smb_const.h"

namespace mrrocpp {
namespace edp {
namespace smb {

/*!
 * \brief class of EDP SwarmItFix mobile base
 *
 * This mobile platform is the base of the SPKM manipulator
 */
class effector: public common::motor_driven_effector
{
protected:
	// Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.

	/*!
	 * \brief method,  creates a list of available kinematic models for smb effector.
	 *
	 * Here it is  motor to joint transform of two legs and manipulator base rotation motor
	 */
	virtual void create_kinematic_models_for_given_robot(void);

public:

	// Konstruktor.
	/*!
	 * \brief
	 *
	 *
	 */
	effector(lib::configurator &_config);

	/*!
	 * \brief
	 *
	 *
	 */
	void create_threads();

	/*!
	 * \brief
	 *
	 *
	 */
	void move_arm(lib::c_buffer &instruction); // przemieszczenie ramienia

	/*!
	 * \brief
	 *
	 *
	 */
	void get_arm_position(bool read_hardware, lib::c_buffer &instruction); // odczytanie pozycji ramienia

	/*!
	 * \brief
	 *
	 *
	 */
	void master_order(common::MT_ORDER nm_task, int nm_tryb);

};

} // namespace smb
} // namespace edp
} // namespace mrrocpp


#endif
