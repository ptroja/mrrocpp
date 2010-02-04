/*!
 * \file edp_e_spkm.h
 * \brief File containing the declaration of edp::spkm::effector class.
 *
 * \author yoyek
 * \date 2009
 *
 */


#ifndef __EDP_E_SPKM_H
#define __EDP_E_SPKM_H

#include "edp/common/edp_e_manip.h"
#include "lib/spkm_const.h"

namespace mrrocpp {
namespace edp {
namespace spkm {

// Klasa reprezentujaca robota IRp-6 na postumencie.
/*!
 * \brief class of EDP SwarmItFix parallel kinematic manipulator
 *
 * It is the base of the head mounted on the mobile base.
 */
class effector: public common::manip_effector
{
protected:

	/*!
	 * \brief sa
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

} // namespace spkm
} // namespace edp
} // namespace mrrocpp


#endif
