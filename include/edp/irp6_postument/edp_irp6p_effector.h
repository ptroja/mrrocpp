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


#ifndef __EDP_IRP6_POSTUMENT_H
#define __EDP_IRP6_POSTUMENT_H

// Klasa edp_irp6s_robot.
#include "edp/common/edp.h"

#define IRP6P_GRIPPER_CATCH_AXE 6
#define IRP6P_GRIPPER_TURN_AXE 5

// Klasa reprezentujaca robota IRp-6 na postumencie.
class edp_irp6p_effector  : public edp_irp6s_postument_track_effector
{
protected:
    // Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
    virtual void create_kinematic_models_for_given_robot(void);

public:
    void initialize (void);
    // Konstruktor.
    edp_irp6p_effector (configurator &_config);

}
; //: edp_irp6p_effector

#endif
