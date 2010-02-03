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
#include "lib/shead_const.h"

namespace mrrocpp {
namespace edp {
namespace shead {

// Klasa reprezentujaca robota IRp-6 na postumencie.


/*!
 * \brief class of EDP SwarmItFix head effector
 *
 * This head is built on top of the SPKM manipulator
 */
class effector: public common::motor_driven_effector
{
protected:
	// Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
	/*!
	 * \brief
	 *
	 *
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
