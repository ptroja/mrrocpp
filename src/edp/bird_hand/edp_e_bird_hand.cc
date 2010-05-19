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
#include "edp/bird_hand/edp_e_bird_hand.h"
#include "edp/common/reader.h"
// Kinematyki.
#include "kinematics/bird_hand/kinematic_model_bird_hand.h"
#include "edp/common/manip_trans_t.h"
#include "edp/common/vis_server.h"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace bird_hand {

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb) {
	manip_effector::single_thread_master_order(nm_task, nm_tryb);
}

void effector::get_controller_state(lib::c_buffer &instruction) {

	// tu wrzucic synchronizacje


	if (test_mode)
		controller_state_edp_buf.is_synchronised = true;
	//printf("get_controller_state: %d\n", controller_state_edp_buf.is_synchronised); fflush(stdout);
	reply.controller_state = controller_state_edp_buf;

	/*
	 // aktualizacja pozycji robota
	 // Uformowanie rozkazu odczytu dla SERVO_GROUP
	 sb->servo_command.instruction_code = lib::READ;
	 // Wyslanie rozkazu do SERVO_GROUP
	 // Pobranie z SERVO_GROUP aktualnej pozycji silnikow
	 //	printf("get_arm_position read_hardware\n");

	 sb->send_to_SERVO_GROUP();
	 */
	// dla pierwszego wypelnienia current_joints
	get_current_kinematic_model()->mp2i_transform(current_motor_pos,
			current_joints);

	{
		boost::mutex::scoped_lock lock(edp_irp6s_effector_mutex);

		// Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
		for (int i = 0; i < number_of_servos; i++) {
			servo_current_motor_pos[i] = desired_motor_pos_new[i]
					= desired_motor_pos_old[i] = current_motor_pos[i];
			desired_joints[i] = current_joints[i];
		}
	}
}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	manip_effector(_config, lib::ROBOT_BIRD_HAND) {
	number_of_servos = BIRD_HAND_NUM_OF_SERVOS;

	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();
}

/*--------------------------------------------------------------------------*/
void effector::move_arm(const lib::c_buffer &instruction) {
	msg->message("move_arm");

	std::stringstream ss(std::stringstream::in | std::stringstream::out);

	ss << ecp_edp_cbuffer.bird_hand_command_structure.desired_position[3];

	msg->message(ss.str().c_str());

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction) {
	//lib::JointArray desired_joints_tmp(MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	//	printf(" GET ARM\n");
	//	flushall();
	static int licznik = (-11);

	std::stringstream ss(std::stringstream::in | std::stringstream::out);
	ss << "get_arm_position: " << licznik;
	msg->message(ss.str().c_str());

	licznik++;

	edp_ecp_rbuffer.bird_hand_status_reply_structure.meassured_current[3]
			= 2.17;

	reply.servo_step = step_counter;

	// clock_gettime();
	// clock_nanosleep();

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::set_robot_model(const lib::c_buffer &instruction) {
	msg->message("set_robot_model");

	std::stringstream ss(std::stringstream::in | std::stringstream::out);

	ss << ecp_edp_cbuffer.bird_hand_configuration_command_structure.d_factor[3];

	msg->message(ss.str().c_str());

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::get_robot_model(lib::c_buffer &instruction) {
	//printf(" GET ROBOT_MODEL: ");
	msg->message("get_robot_model");

	edp_ecp_rbuffer.bird_hand_configuration_reply_structure.d_factor[2] = 121;

}
/*--------------------------------------------------------------------------*/

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void) {
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::bird_hand::kinematic_model_bird_hand());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

/*--------------------------------------------------------------------------*/
void effector::create_threads() {
	rb_obj = new common::reader_buffer(*this);
	vis_obj = new common::vis_server(*this);
}

lib::INSTRUCTION_TYPE effector::receive_instruction(void) {

	lib::INSTRUCTION_TYPE ret_val = common::effector::receive_instruction();
	memcpy(&ecp_edp_cbuffer, instruction.arm.serialized_command,
			sizeof(ecp_edp_cbuffer));
	return ret_val;
}

void effector::reply_to_instruction(void) {
	memcpy(reply.arm.serialized_reply, &edp_ecp_rbuffer,
			sizeof(edp_ecp_rbuffer));
	common::effector::reply_to_instruction();
}

}
// namespace bird_hand


namespace common {

// Stworzenie obiektu edp_irp6m_effector.
effector* return_created_efector(lib::configurator &_config) {
	return new bird_hand::effector(_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

