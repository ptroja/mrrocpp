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

#include "base/edp/edp_e_manip.h"
#include "robot/irp6p_m/const_irp6p_m.h"

#include <boost/thread.hpp>

namespace mrrocpp {
namespace edp {
namespace irp6p_m {

const int IRP6P_GRIPPER_TURN_AXE = 5;

const double AXIS_0_TO_5_INC_PER_REVOLUTION = 4000; // Liczba impulsow enkodera na obrot walu - musi byc float
const double AXIS_6_INC_PER_REVOLUTION = 2000; // Liczba impulsow enkodera na obrot walu - musi byc float

// Klasa reprezentujaca robota IRp-6 na postumencie.
class effector : public common::manip_effector
{
protected:
	// Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
	virtual void create_kinematic_models_for_given_robot(void);

	boost::thread thread_id;

public:
	effector(common::shell &_shell);

	~effector();

	common::servo_buffer *return_created_servo_buffer();

	void set_robot_model(const lib::c_buffer &);
	void create_threads();
	void move_arm(const lib::c_buffer &);
	void get_arm_position(bool, lib::c_buffer &);

	void master_order(common::MT_ORDER nm_task, int nm_tryb);

	/*!
	 * \brief The particular type of instruction send form ECP to EDP
	 */
	lib::c_buffer instruction;

	/*!
	 * \brief The particular type of reply send form EDP to ECP
	 */
	lib::r_buffer reply;

};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
