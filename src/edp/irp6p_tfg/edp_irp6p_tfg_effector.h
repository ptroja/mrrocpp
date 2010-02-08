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


#ifndef __EDP_IRP6P_TFG_H
#define __EDP_IRP6P_TFG_H

// Klasa edp_irp6s_robot.
#include "edp/irp6p_tfg/sg_irp6p_tfg.h"
#include "edp/common/edp_irp6s_postument_track.h"
#include "lib/irp6p_tfg_const.h"

#define IRP6P_GRIPPER_CATCH_AXE 6
#define IRP6P_GRIPPER_TURN_AXE 5

namespace mrrocpp {
namespace edp {
namespace irp6p {

// Klasa reprezentujaca robota IRp-6 na postumencie.
class effector : public common::irp6s_postument_track_effector
{
protected:
    // Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
    virtual void create_kinematic_models_for_given_robot(void);

public:
    effector (lib::configurator &_config);
    common::servo_buffer *return_created_servo_buffer ();
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
