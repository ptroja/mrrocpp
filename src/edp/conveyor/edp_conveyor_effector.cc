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

using namespace mrrocpp::edp::common::exception;

namespace mrrocpp {
namespace edp {
namespace conveyor {

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
			set_rmodel_servo_algorithm(instruction);
			break;

		default: // blad: nie istniejca specyfikacja modelu robota
			// ustawi numer bledu
			manip_and_conv_effector::set_rmodel(instruction);
	}
}
/*--------------------------------------------------------------------------*/

// servo_joints_and_frame_actualization_and_upload.
void effector::servo_joints_and_frame_actualization_and_upload(void)
{
	static int catch_nr = 0;
	// wyznaczenie nowych wartosci joints and frame dla obliczen w servo
	try {
		get_current_kinematic_model()->mp2i_transform(servo_current_motor_pos, servo_current_joints);
		catch_nr = 0;
	}//: try
	catch (...) {
		if ((++catch_nr) == 1)
			printf("servo thread servo_joints_and_frame_actualization_and_upload throw catch exception\n");
	}//: catch

	{
		boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);

		// przepisnie danych na zestaw globalny
		for (int i = 0; i < number_of_servos; i++) {
			global_current_motor_pos[i] = servo_current_motor_pos[i];
			global_current_joints[i] = servo_current_joints[i];
		}//: for

	}
}//: servo_joints_and_frame_actualization_and_upload


// Przemieszczenie tasmociagu.
void effector::move_arm(lib::c_buffer &instruction)
{
	manip_and_conv_effector::multi_thread_move_arm(instruction);
}

// Odczytanie pozycji tasmociagu.
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{

	lib::JointArray desired_joints_tmp(MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	if (read_hardware) {
		// Uformowanie rozkazu odczytu dla SERVO_GROUP
		sb->servo_command.instruction_code = lib::READ;
		// Wyslanie rozkazu do SERVO_GROUP
		// Pobranie z SERVO_GROUP aktualnej pozycji silnikow
		//		printf("get_arm_position read_hardware\n");

		sb->send_to_SERVO_GROUP();

		// Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
		for (int i = 0; i < number_of_servos; i++) {
			desired_motor_pos_old[i] = current_motor_pos[i];
		}
		if (is_synchronised()) {
			//  check_motor_position(desired_motor_pos_new);
			// dla sprawdzenia ograncizen w joints i motors

			get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, desired_joints_tmp);

			for (int i = 0; i < number_of_servos; i++) {
				desired_joints[i] = current_joints[i] = desired_joints_tmp[i];
			}
		}
	}

	// okreslenie rodzaju wspolrzednych, ktore maja by odczytane
	// oraz adekwatne wypelnienie bufora odpowiedzi
	switch (instruction.get_arm_type)
	{
		case lib::JOINT:
			// przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
			get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
			// przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
			// Przepisanie definicji koncowki danej w postaci
			// JOINTS z wewntrznych struktur danych TRANSFORMATORa
			// do wewntrznych struktur danych REPLY_BUFFER
			reply.arm_type = lib::JOINT;
			for (int i = 0; i < number_of_servos; i++) {
				reply.arm.pf_def.arm_coordinates[i] = current_joints[i];
			}
			break;
		case lib::MOTOR:
			reply.arm_type = lib::MOTOR;
			for (int i = 0; i < number_of_servos; i++) {
				reply.arm.pf_def.arm_coordinates[i] = current_motor_pos[i];
			}
			break;
		default: // blad: nieznany sposob zapisu wspolrzednych koncowki
			printf("EFF_TYPE: %d\n", instruction.get_arm_type);
			throw NonFatal_error_2(INVALID_GET_END_EFFECTOR_TYPE);
	}

	// scope-locked reader data update
	{
		boost::mutex::scoped_lock lock(rb_obj->reader_mutex);

		reply.servo_step = rb_obj->step_data.step;
	}
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
