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

#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"
#include "lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "edp/smb/edp_e_smb.h"
#include "edp/common/reader.h"
// Kinematyki.
#include "kinematics/smb/kinematic_model_smb.h"
#include "edp/common/manip_trans_t.h"
#include "edp/common/vis_server.h"

#include "edp/common/exception.h"
using namespace mrrocpp::edp::common::exception;

namespace mrrocpp {
namespace edp {
namespace smb {

common::servo_buffer* effector::return_created_servo_buffer(void)
{
	assert(0);
	return NULL;
}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	manip_effector::single_thread_master_order(nm_task, nm_tryb);
}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	manip_effector(_config, lib::ROBOT_SMB)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	number_of_servos = SMB_NUM_OF_SERVOS;

	reset_variables();
}


/*--------------------------------------------------------------------------*/
void effector::move_arm(lib::c_buffer &instruction)
{
	manip_effector::single_thread_move_arm(instruction);

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{ // odczytanie pozycji ramienia

	//   printf(" GET ARM\n");
	//lib::JointArray desired_joints_tmp(MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	if (read_hardware) {
	//	manip_and_conv_effector::get_arm_position_read_hardware_sb();
	}

	// okreslenie rodzaju wspolrzednych, ktore maja by odczytane
	// oraz adekwatne wypelnienie bufora odpowiedzi
	common::manip_effector::get_arm_position_get_arm_type_switch(instruction);

	// scope-locked reader data update
	{
		boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

		reply.servo_step = rb_obj->step_data.step;
	}
}
/*--------------------------------------------------------------------------*/

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::smb::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

void effector::servo_joints_and_frame_actualization_and_upload(void)
{

}

/*--------------------------------------------------------------------------*/
void effector::create_threads()
{
	rb_obj = new common::reader_buffer(*this);
	vis_obj = new common::vis_server(*this);
}

}
// namespace smb


namespace common {

// Stworzenie obiektu edp_irp6m_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new smb::effector(_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

