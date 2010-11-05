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


#ifndef __EDP_E_POLYCRANK_H
#define __EDP_E_POLYCRANK_H

#include "base/edp/edp_e_manip.h"
#include "robot/polycrank/const_polycrank.h"

namespace mrrocpp {
namespace edp {
namespace polycrank {

// Klasa reprezentujaca robota IRp-6 na postumencie.
class effector: public common::manip_effector
{
protected:
	// Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
	virtual void create_kinematic_models_for_given_robot(void);

public:

	// Konstruktor.
	effector(lib::configurator &_config);

	void create_threads();

	void move_arm(const lib::c_buffer &instruction); // przemieszczenie ramienia

	void get_arm_position(bool read_hardware, lib::c_buffer &instruction); // odczytanie pozycji ramienia
	void master_order(common::MT_ORDER nm_task, int nm_tryb);
};

} // namespace polycrank
} // namespace edp
} // namespace mrrocpp


#endif
