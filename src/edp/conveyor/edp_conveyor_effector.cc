// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6ot_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robota IRp-6 na torze jezdnym
//				- definicja metod klasy edp_irp6ot_effector
//				- definicja funkcji return_created_efector()
//
// Autor:		tkornuta
// Data:		24.02.2007
// ------------------------------------------------------------------------

#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mrmath/mrmath.h"
#include "edp/common/reader.h"

// Klasa edp_conveyor_effector.
#include "edp/common/edp.h"
#include "edp/common/manip_trans_t.h"
#include "edp/conveyor/edp_conveyor_effector.h"
#include "edp/common/servo_gr.h"
// Model kinematyczny tasmociagu.
#include "kinematics/conveyor/kinematic_model_conveyor.h"
#include "lib/conveyor_const.h"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace conveyor {


/*--------------------------------------------------------------------------*/
void effector::create_threads()
{
	manip_and_conv_effector::hi_create_threads();
}


common::servo_buffer* effector::return_created_servo_buffer()
{
	return new conveyor::servo_buffer(*this);
}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	manip_and_conv_effector::multi_thread_master_order(nm_task, nm_tryb);
}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	manip_and_conv_effector(_config, lib::ROBOT_CONVEYOR)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	number_of_servos = CONVEYOR_NUM_OF_SERVOS;

	reset_variables();
}

/*--------------------------------------------------------------------------*/
void effector::set_rmodel(lib::c_buffer &instruction)
{
	// uint8_t previous_model;
	// uint8_t previous_corrector;
	//printf(" SET RMODEL: ");
	switch (instruction.set_rmodel_type)
	{
		case lib::SERVO_ALGORITHM:
			sb->set_rmodel_servo_algorithm(instruction);
			break;

		default: // blad: nie istniejca specyfikacja modelu robota
			// ustawi numer bledu
			manip_and_conv_effector::set_rmodel(instruction);
	}
}
/*--------------------------------------------------------------------------*/


// Przemieszczenie tasmociagu.
void effector::move_arm(lib::c_buffer &instruction)
{
	manip_and_conv_effector::multi_thread_move_arm(instruction);
}

// Odczytanie pozycji tasmociagu.
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{

	//lib::JointArray desired_joints_tmp(MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	if (read_hardware) {
		manip_and_conv_effector::get_arm_position_read_hardware_sb();
	}

	// okreslenie rodzaju wspolrzednych, ktore maja by odczytane
	// oraz adekwatne wypelnienie bufora odpowiedzi
	common::manip_and_conv_effector::get_arm_position_get_arm_type_switch(instruction);

	manip_and_conv_effector::get_arm_position_set_reply_step();
}

// Stworzenie modeli kinematyki dla tasmociagu.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::conveyor::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}//: create_kinematic_models_for_given_robot

} // namespace conveyor

namespace common {

// Stworzenie obiektu edp_conveyor_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new conveyor::effector(_config);
}//: return_created_efector

} // namespace common
} // namespace edp
} // namespace mrrocpp
