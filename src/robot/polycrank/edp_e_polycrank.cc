// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6m_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- definicja metod klasy edp_irp6m_effector
//				- definicja funkcji return_created_efector()
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "robot/polycrank/edp_e_polycrank.h"
#include "base/edp/reader.h"
// Kinematyki.
#include "robot/polycrank/kinematic_model_polycrank.h"
#include "base/edp/manip_trans_t.h"
#include "base/edp/vis_server.h"

#include "base/lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace polycrank {

// Konstruktor.
effector::effector(lib::configurator &_config) :
	manip_effector(_config, lib::polycrank::ROBOT_NAME)
{
	number_of_servos = lib::polycrank::NUM_OF_SERVOS;

	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();
}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	manip_effector::single_thread_master_order(nm_task, nm_tryb);
}

/*--------------------------------------------------------------------------*/
void effector::move_arm(const lib::c_buffer &instruction)
{
	manip_effector::single_thread_move_arm(instruction);
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{ // odczytanie pozycji ramienia

	//   printf(" GET ARM\n");
	//lib::JointArray desired_joints_tmp(lib::MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -

	if (read_hardware) {
		//	motor_driven_effector::get_arm_position_read_hardware_sb();
	}

	// okreslenie rodzaju wspolrzednych, ktore maja by odczytane
	// oraz adekwatne wypelnienie bufora odpowiedzi
	common::manip_effector::get_arm_position_get_arm_type_switch(instruction);

	reply.servo_step = step_counter;
}
/*--------------------------------------------------------------------------*/

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::polycrank::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

/*--------------------------------------------------------------------------*/
void effector::create_threads()
{
	rb_obj = (boost::shared_ptr<common::reader_buffer>) new common::reader_buffer(*this);
	vis_obj = (boost::shared_ptr<common::vis_server>) new common::vis_server(*this);
}

}
// namespace polycrank


namespace common {

// Stworzenie obiektu edp_irp6m_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new polycrank::effector(_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

