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

#include "edp/common/edp.h"

// Klasa reprezentujaca tasmociag.
class edp_conveyor_effector  : public edp_irp6s_and_conv_effector
{
protected:
    // Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
    virtual void create_kinematic_models_for_given_robot(void);

public:
    // Konstruktor.
    edp_conveyor_effector (configurator &_config);
    void initialize (void);
    void set_rmodel (c_buffer *instruction);                    // zmiana narzedzia
    void get_rmodel (c_buffer *instruction);                    // odczytanie narzedzia

    // Przemieszczenie ramienia.
    void move_arm (const c_buffer &instruction);
    // Odczytanie pozycji ramienia.
    void get_arm_position (bool read_hardware, c_buffer *instruction);
    // Aktualizacja polozenia.
    void servo_joints_and_frame_actualization_and_upload(void);

}
; //: edp_conveyor_effector

#endif
