// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6p_effector.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- deklaracja klasy edp_irp6p_effector
//
// Autor:
// Data:		17.01.2007
// ------------------------------------------------------------------------


#ifndef __EDP_E_SPKM_H
#define __EDP_E_SPKM_H

#include "edp/common/edp_e_manip.h"
#include "lib/spkm_const.h"

namespace mrrocpp {
namespace edp {
namespace spkm {

// Klasa reprezentujaca robota IRp-6 na postumencie.
class effector : public common::manip_effector
{
protected:
    // Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
    virtual void create_kinematic_models_for_given_robot(void);





public:

    void set_rmodel (lib::c_buffer &instruction);                    // zmiana narzedzia
    void get_rmodel (lib::c_buffer &instruction);                    // odczytanie narzedzia

    // Konstruktor.
    effector (lib::configurator &_config);

    void create_threads ();

    void servo_joints_and_frame_actualization_and_upload(void);// by Y

    void move_arm (lib::c_buffer &instruction);            // przemieszczenie ramienia

    void get_arm_position (bool read_hardware, lib::c_buffer &instruction); // odczytanie pozycji ramienia
    void master_order(common::MT_ORDER nm_task, int nm_tryb);

    common::servo_buffer* return_created_servo_buffer(void);
};

} // namespace spkm
} // namespace edp
} // namespace mrrocpp



#endif
