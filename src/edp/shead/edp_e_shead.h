// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6p_effector.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- deklaracja klasy edp_irp6p_effector
//
// Autor:		tkornuta
// Data:		17.01.2007
// ------------------------------------------------------------------------


#ifndef __EDP_E_SHEAD_H
#define __EDP_E_SHEAD_H

#include "edp/common/edp_e_motor_driven.h"
#include "lib/shead_const.h"

namespace mrrocpp {
namespace edp {
namespace shead {

// Klasa reprezentujaca robota IRp-6 na postumencie.
class effector: public common::motor_driven_effector
{
protected:
	// Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
	virtual void create_kinematic_models_for_given_robot(void);

public:

	// Konstruktor.
	effector(lib::configurator &_config);

	void create_threads();

	void move_arm(lib::c_buffer &instruction); // przemieszczenie ramienia

	void get_arm_position(bool read_hardware, lib::c_buffer &instruction); // odczytanie pozycji ramienia
	void master_order(common::MT_ORDER nm_task, int nm_tryb);

};

} // namespace smb
} // namespace edp
} // namespace mrrocpp


#endif
