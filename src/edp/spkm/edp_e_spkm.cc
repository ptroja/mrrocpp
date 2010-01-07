// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6m_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot IRp-6 na postumencie
//				- definicja metod klasy edp_irp6m_effector
//				- definicja funkcji return_created_efector()
//
// Autor:
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"
#include "lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "edp/spkm/edp_e_spkm.h"
#include "edp/common/reader.h"
// Kinematyki.
#include "kinematics/spkm/kinematic_model_spkm.h"
#include "edp/common/manip_trans_t.h"
#include "edp/common/vis_server.h"

using namespace mrrocpp::edp::common::exception;

namespace mrrocpp {
namespace edp {
namespace spkm {

common::servo_buffer* effector::return_created_servo_buffer(void)
{
	// TODO
	assert(0);
	return NULL;
}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	manip_effector::single_thread_master_order(nm_task, nm_tryb);
}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	manip_effector(_config, lib::ROBOT_SPKM)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	number_of_servos = SPKM_NUM_OF_SERVOS;

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
	lib::JointArray desired_joints_tmp(MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	//   printf(" GET ARM\n");

	if (read_hardware) {
		// Uformowanie rozkazu odczytu dla SERVO_GROUP
		//       servo_command.instruction_code = lib::READ;
		// Wyslanie rozkazu do SERVO_GROUP
		// Pobranie z SERVO_GROUP aktualnej pozycji silnikow
		//		printf("get_arm_position read_hardware\n");

		//     send_to_SERVO_GROUP ();

		// Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
		for (int i = 0; i < number_of_servos; i++) {
			desired_motor_pos_new[i] = desired_motor_pos_old[i] = current_motor_pos[i];
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
		case lib::FRAME:
			// przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
			get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
			get_current_kinematic_model()->i2e_transform(current_joints, current_end_effector_frame);
			// TRANS z wewntrznych struktur danych TRANSFORMATORa
			// do wewntrznych struktur danych REPLY_BUFFER
			current_end_effector_frame.get_frame_tab(reply.arm.pf_def.arm_frame);
			break;

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
/*--------------------------------------------------------------------------*/

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::spkm::kinematic_model_spkm());
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
// namespace spkm


namespace common {

// Stworzenie obiektu edp_irp6m_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new spkm::effector(_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

