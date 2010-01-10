// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_conveyor_effector.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Tasmociag
//				- deklaracja klasy edp_conveyor_effector
//
// Autor:		tkornuta
// Data:		17.01.2007
// ------------------------------------------------------------------------

#ifndef __edp_conveyor_effector_H
#define __edp_conveyor_effector_H

#include "edp/conveyor/sg_conv.h"
#include "edp/common/edp_e_manip_and_conv.h"
#include "lib/conveyor_const.h"

namespace mrrocpp {
namespace edp {
namespace conveyor {

// Klasa reprezentujaca tasmociag.
class effector  : public common::manip_and_conv_effector
{
protected:
    // Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
    virtual void create_kinematic_models_for_given_robot(void);

public:
    // Konstruktor.
    effector (lib::configurator &_config);

    void set_rmodel (lib::c_buffer &instruction);                    // zmiana narzedzia

    void create_threads();

    // Przemieszczenie ramienia.
    void move_arm (lib::c_buffer &instruction);
    // Odczytanie pozycji ramienia.
    void get_arm_position (bool read_hardware, lib::c_buffer &instruction);
    // Aktualizacja polozenia.
    void servo_joints_and_frame_actualization_and_upload(void);

    common::servo_buffer *return_created_servo_buffer ();
	void master_order(common::MT_ORDER nm_task, int nm_tryb);

};

} // namespace conveyor
} // namespace edp
} // namespace mrrocpp


#endif
