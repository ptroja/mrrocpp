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
#include "edp/common/edp.h"

namespace mrrocpp {
namespace edp {
namespace irp6m {

// Klasa reprezentujaca robota IRp-6 na postumencie.
class effector : public common::irp6s_effector
{
protected:
    // Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
    virtual void create_kinematic_models_for_given_robot(void);

    void arm_abs_xyz_eul_zyz_2_frame (const double *p);
    // Przeksztalcenie definicji koncowki z postaci
    // XYZ_EULER_ZYZ wyrazonej bezwzglednie do postaci
    // FRAME oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych TRANSFORMATORa

    void arm_frame_2_xyz_eul_zyz ();

public:

    void set_rmodel (c_buffer &instruction);                    // zmiana narzedzia
    void get_rmodel (c_buffer &instruction);                    // odczytanie narzedzia
    // Konstruktor.
    void initialize (void);
    effector (configurator &_config);

    void servo_joints_and_frame_actualization_and_upload(void);// by Y

    void move_arm (c_buffer &instruction);            // przemieszczenie ramienia

    void get_arm_position (bool read_hardware, c_buffer &instruction); // odczytanie pozycji ramienia
};

} // namespace common
} // namespace edp
} // namespace mrrocpp



#endif
