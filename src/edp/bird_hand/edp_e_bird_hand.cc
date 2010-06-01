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

#define PORT "/dev/ser"

using namespace mrrocpp::lib::exception;

#ifndef __QNXNTO__

uint64_t timespec2nsec(const timespec *t) {
	return t->tv_sec * 1000000 + t->tv_nsec;
}

void nsec2timespec(timespec *t, uint64_t nsec) {
	t->tv_sec = nsec / 1000000;
	t->tv_nsec = nsec % 1000000;
}

#endif

namespace mrrocpp {
namespace edp {
namespace bird_hand {

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb) {
	manip_effector::single_thread_master_order(nm_task, nm_tryb);
}

void effector::get_controller_state(lib::c_buffer &instruction) {

	if (!test_mode) {
		for (uint8_t i = 0; i < BIRD_HAND_NUM_OF_SERVOS; i++) {
			int16_t abspos;
			if (i < 2)
				device.getSynchroPos(i, abspos);
			synchro_position[i] = (int32_t) (abspos * 124345.23443); // TODO : tu wstawić magiczną stałą
		}
	}
	controller_state_edp_buf.is_synchronised = true;

	printf("synchro position readed : %d \n", synchro_position[0]);

	reply.controller_state = controller_state_edp_buf;

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

	// inicjacja czasow

	struct timespec current_timespec;

	if (clock_gettime(CLOCK_MONOTONIC, &current_timespec) == -1) {
		perror("clock gettime");
	}

	macrostep_end_time = timespec2nsec(&current_timespec);

}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	manip_effector(_config, lib::ROBOT_BIRD_HAND) {
	number_of_servos = BIRD_HAND_NUM_OF_SERVOS;

	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();
	if (!test_mode)
		device.connect(PORT);
}

/*--------------------------------------------------------------------------*/
void effector::move_arm(const lib::c_buffer &instruction) {

	struct timespec current_timespec;
	if (!test_mode) {
		for (unsigned int i = 0; i < BIRD_HAND_NUM_OF_SERVOS; i++) {
			switch (ecp_edp_cbuffer.bird_hand_command_structure.finger[i].profile_type) {
			case mrrocpp::lib::BIRD_HAND_SIGLE_STEP_POSTION_INCREMENT:
				device.setCMD1(
						(uint16_t) i,
						(int16_t) ecp_edp_cbuffer.bird_hand_command_structure.motion_steps,
						(int16_t) ecp_edp_cbuffer.bird_hand_command_structure.finger[i].reciprocal_of_damping,
						(int16_t) ecp_edp_cbuffer.bird_hand_command_structure.finger[i].desired_torque,
						(int32_t) ecp_edp_cbuffer.bird_hand_command_structure.finger[i].desired_position);
				break;
			case mrrocpp::lib::BIRD_HAND_MACROSTEP_POSITION_INCREMENT:
				device.setCMD2(
						(uint16_t) i,
						(int16_t) ecp_edp_cbuffer.bird_hand_command_structure.motion_steps,
						(int16_t) ecp_edp_cbuffer.bird_hand_command_structure.finger[i].reciprocal_of_damping,
						(int16_t) ecp_edp_cbuffer.bird_hand_command_structure.finger[i].desired_torque,
						(int32_t) ecp_edp_cbuffer.bird_hand_command_structure.finger[i].desired_position);
				break;
			case mrrocpp::lib::BIRD_HAND_MACROSTEP_ABSOLUTE_POSITION:
				device.setCMD3(
						(uint16_t) i,
						(int16_t) ecp_edp_cbuffer.bird_hand_command_structure.motion_steps,
						(int16_t) ecp_edp_cbuffer.bird_hand_command_structure.finger[i].reciprocal_of_damping,
						(int16_t) ecp_edp_cbuffer.bird_hand_command_structure.finger[i].desired_torque,
						(int32_t) ecp_edp_cbuffer.bird_hand_command_structure.finger[i].desired_position);
				break;
			}
		}
	}

	std::stringstream ss(std::stringstream::in | std::stringstream::out);
	/*
	 ss << "dt: "
	 << ecp_edp_cbuffer.bird_hand_command_structure.finger[2].desired_torque
	 << " rod: "
	 << ecp_edp_cbuffer.bird_hand_command_structure.finger[2].reciprocal_of_damping
	 << " dp: "
	 << ecp_edp_cbuffer.bird_hand_command_structure.finger[2].desired_position;

	 msg->message(ss.str().c_str());
	 */
	if (clock_gettime(CLOCK_MONOTONIC, &current_timespec) == -1) {
		perror("clock gettime");
	}

	uint64_t current_time = timespec2nsec(&current_timespec);

	if (current_time >= macrostep_end_time) {
		// stan bierny


		query_time = current_time
				+ ecp_edp_cbuffer.bird_hand_command_structure.ecp_query_step
						* BIRD_HAND_STEP_TIME_IN_NS;

		macrostep_end_time = current_time
				+ ecp_edp_cbuffer.bird_hand_command_structure.motion_steps
						* BIRD_HAND_STEP_TIME_IN_NS;

		/*
		 ss << " stan bierny c:" << current_time << " q: " << query_time
		 << " e: " << macrostep_end_time;

		 msg->message(ss.str().c_str());
		 ss.str("");

		 ss << " stan bierny c:" << current_time << " q: "
		 << ecp_edp_cbuffer.bird_hand_command_structure.ecp_query_step
		 << " e: "
		 << ecp_edp_cbuffer.bird_hand_command_structure.motion_steps;

		 msg->message(ss.str().c_str());

		 ss.str("");

		 ss << " stan bierny c:" << current_time << " q: "
		 << ecp_edp_cbuffer.bird_hand_command_structure.ecp_query_step
		 * (uint64_t) BIRD_HAND_STEP_TIME_IN_NS << " e: "
		 << ecp_edp_cbuffer.bird_hand_command_structure.motion_steps
		 * (uint64_t) BIRD_HAND_STEP_TIME_IN_NS;

		 msg->message(ss.str().c_str());
		 */
	} else {
		// stan czynny
		// UWAGA NA KOLEJNOSC OBLICZEN query_time i macrostep_end_time NIE ZAMIENIAC
		/*
		 msg->message("stan czynny");
		 */
		query_time = macrostep_end_time
				+ ecp_edp_cbuffer.bird_hand_command_structure.ecp_query_step
						* BIRD_HAND_STEP_TIME_IN_NS;

		macrostep_end_time
				+= ecp_edp_cbuffer.bird_hand_command_structure.motion_steps
						* BIRD_HAND_STEP_TIME_IN_NS;

	}

	device.synchronize(255, 40);

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction) {

	struct timespec query_timespec;

	nsec2timespec(&query_timespec, query_time);

	// zawieszenie do query_time

	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &query_timespec, NULL);

	if (test_mode) {
		for (int i = 0; i < BIRD_HAND_NUM_OF_SERVOS; i++) {
			edp_ecp_rbuffer.bird_hand_status_reply_structure.finger[i].meassured_position
					= ecp_edp_cbuffer.bird_hand_command_structure.finger[i].desired_position;
			edp_ecp_rbuffer.bird_hand_status_reply_structure.finger[i].meassured_torque
					= ecp_edp_cbuffer.bird_hand_command_structure.finger[i].desired_torque;
		}
	} else {
		for (uint8_t i = 0; i < BIRD_HAND_NUM_OF_SERVOS; i++) {
			int32_t pos;
			int16_t t, c;
			uint8_t status;

			if (i < 2)
				device.getStatus(i, status, pos, c, t);

			edp_ecp_rbuffer.bird_hand_status_reply_structure.finger[i].meassured_position
					= (pos - synchro_position[i]);
			edp_ecp_rbuffer.bird_hand_status_reply_structure.finger[i].meassured_current
					= c;
			edp_ecp_rbuffer.bird_hand_status_reply_structure.finger[i].meassured_torque
					= t;
		}
	}
	reply.servo_step = step_counter;

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::set_robot_model(const lib::c_buffer &instruction) {

	for (uint8_t i = 0; i < BIRD_HAND_NUM_OF_SERVOS; i++) {
		int16_t p, i, d;
		p
				= edp_ecp_rbuffer.bird_hand_configuration_reply_structure.finger[i].p_factor;
		i
				= edp_ecp_rbuffer.bird_hand_configuration_reply_structure.finger[i].i_factor;
		d
				= edp_ecp_rbuffer.bird_hand_configuration_reply_structure.finger[i].d_factor;
		if (i < 2)
			device.setPID(i, p, i, d);

		//TODO : add limits
	}
}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::get_robot_model(lib::c_buffer &instruction) {
	if (!test_mode) {

		for (uint8_t i = 0; i < BIRD_HAND_NUM_OF_SERVOS; i++) {
			int16_t p, i, d;

			if (i < 2)
				device.getPID(i, p, i, d);
			edp_ecp_rbuffer.bird_hand_configuration_reply_structure.finger[i].p_factor
					= p;
			edp_ecp_rbuffer.bird_hand_configuration_reply_structure.finger[i].i_factor
					= i;
			edp_ecp_rbuffer.bird_hand_configuration_reply_structure.finger[i].d_factor
					= d;

			//TODO : add limits
		}
	}
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

void effector::instruction_deserialization() {

	memcpy(&ecp_edp_cbuffer, instruction.arm.serialized_command,
			sizeof(ecp_edp_cbuffer));

}

void effector::reply_serialization(void) {
	memcpy(reply.arm.serialized_reply, &edp_ecp_rbuffer,
			sizeof(edp_ecp_rbuffer));

}

}
// namespace bird_hand


namespace common {

// Stworzenie obiektu edp_bird_hand_effector.
effector* return_created_efector(lib::configurator &_config) {
	return new bird_hand::effector(_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

