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


#ifndef __EDP_IRP6P_M_H
#define __EDP_IRP6P_M_H

// Klasa edp_irp6s_robot.
#include "edp/irp6p_m/sg_irp6p_m.h"
#include "edp/common/edp_e_manip.h"
#include "lib/robot_consts/irp6p_m_const.h"

#define IRP6P_GRIPPER_TURN_AXE 5

namespace mrrocpp {
namespace edp {
namespace irp6p_m {

// Klasa reprezentujaca robota IRp-6 na postumencie.
class effector: public common::manip_effector
{
protected:
	// Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
	virtual void create_kinematic_models_for_given_robot(void);

public:
	effector(lib::configurator &_config);
	common::servo_buffer *return_created_servo_buffer();

    void set_robot_model (const lib::c_buffer &);
    void create_threads ();
    void move_arm (const lib::c_buffer &);
    void get_arm_position(bool, lib::c_buffer &);

    void master_order(common::MT_ORDER nm_task, int nm_tryb);

};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
