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


#ifndef __EDP_IRP6_MECHATRONIKA_H
#define __EDP_IRP6_MECHATRONIKA_H

// Klasa edp_irp6s_robot.
#include "edp/irp6_mechatronika/sg_irp6m.h"
#include "edp/common/edp_e_manip.h"
#include "lib/irp6m_const.h"

namespace mrrocpp {
namespace edp {
namespace irp6m {

// Klasa reprezentujaca robota IRp-6 na postumencie.
class effector : public common::manip_effector
{
protected:
    // Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
    virtual void create_kinematic_models_for_given_robot(void);





public:

    void set_rmodel (lib::c_buffer &instruction);                    // zmiana narzedzia


    // Konstruktor.
    effector (lib::configurator &_config);

    void servo_joints_and_frame_actualization_and_upload(void);// by Y

    void move_arm (lib::c_buffer &instruction);            // przemieszczenie ramienia

    void get_arm_position (bool read_hardware, lib::c_buffer &instruction); // odczytanie pozycji ramienia

    common::servo_buffer *return_created_servo_buffer ();
    void master_order(common::MT_ORDER nm_task, int nm_tryb);
};

} // namespace common
} // namespace edp
} // namespace mrrocpp



#endif
