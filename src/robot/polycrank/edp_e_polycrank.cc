// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:		edp_e_polycrank.cc
// System:		QNX/MRROC++  v. 6.5
// Opis:		Robot polycrank
//				- definicja metod klasy edp_e_polycrank
//				- definicja funkcji return_created_efector()
//
// Autor:		Mariusz Zbikowski
// Data:		13.01.2011
// ------------------------------------------------------------------------

//#include <boost/thread/thread.hpp>
//#include <boost/bind.hpp>

#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/edp/reader.h"

// Klasa edp_e_polycrank.
#include "base/edp/edp_typedefs.h"
#include "base/edp/manip_trans_t.h"
#include "robot/polycrank/edp_e_polycrank.h"
#include "base/edp/servo_gr.h"

// Kinematyki.
#include "robot/polycrank/kinematic_model_polycrank.h"
#include "robot/polycrank/const_polycrank.h"

#include "base/edp/vis_server.h"
#include "base/lib/exception.h"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace polycrank {

void effector::create_threads()
{
	motor_driven_effector::hi_create_threads();
}

common::servo_buffer* effector::return_created_servo_buffer()
{
	return new polycrank::servo_buffer(*this);
}

// Konstruktor.
effector::effector(common::shell &_shell) :
	//manip_effector(_shell, lib::polycrank::ROBOT_NAME)
	motor_driven_effector(_shell, lib::polycrank::ROBOT_NAME)
{
	number_of_servos = lib::polycrank::NUM_OF_SERVOS;

	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();
}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	//printf("master_order\n");
	//motor_driven_effector::multi_thread_master_order(nm_task, nm_tryb);
	motor_driven_effector::single_thread_master_order(nm_task, nm_tryb);
}

void effector::set_robot_model(const lib::c_buffer &instruction)
{
	// uint8_t previous_model;
	// uint8_t previous_corrector;
	//printf(" SET ROBOT_MODEL: ");
	switch (instruction.robot_model.type)
	{
		case lib::SERVO_ALGORITHM:
			sb->set_robot_model_servo_algorithm(instruction);
			break;

		default: // blad: nie istniejca specyfikacja modelu robota
			// ustawi numer bledu
			motor_driven_effector::set_robot_model(instruction);
	}
}

// Przemieszczenie tasmociagu-conveyor MOZE BYC SINGLE LUB MULTI
void effector::move_arm(const lib::c_buffer &instruction)
{
	//printf("move arm \n");
	//manip_effector::multi_thread_move_arm(instruction);
	//motor_driven_effector::single_thread_move_arm(instruction);
	motor_driven_effector::multi_thread_move_arm(instruction);
}

// Odczytanie pozycji tasmociagu - conveyor
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{
	//printf("get_arm_position \n");
	//lib::JointArray desired_joints_tmp(lib::MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -

	if (read_hardware) {
		motor_driven_effector::get_arm_position_read_hardware_sb();
	}

	common::motor_driven_effector::get_arm_position_get_arm_type_switch(instruction);

	reply.servo_step = step_counter;
}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void)
{
	//printf("create_kinematic_models_for_given_robot \n");

	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::polycrank::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

}
// namespace polycrank

namespace common {


effector* return_created_efector(common::shell &_shell)
{
	return new polycrank::effector(_shell);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

