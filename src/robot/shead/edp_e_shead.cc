#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "robot/shead/edp_e_shead.h"
#include "base/edp/reader.h"
// Kinematyki.
#include "robot/shead/kinematic_model_shead.h"
#include "base/edp/manip_trans_t.h"
#include "base/edp/vis_server.h"

#include "base/lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace shead {

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	motor_driven_effector::single_thread_master_order(nm_task, nm_tryb);
}

// Konstruktor.
effector::effector(common::shell &_shell, lib::robot_name_t l_robot_name) :
	motor_driven_effector(_shell, l_robot_name)
{

	number_of_servos = lib::shead::NUM_OF_SERVOS;
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();
}

void effector::synchronise(void)
{
	try {
		if (robot_test_mode) {
			controller_state_edp_buf.is_synchronised = true;
			return;
		}

	} catch (mrrocpp::lib::exception::non_fatal_error & e_) {
		// Standard error handling.
		HANDLE_EDP_NON_FATAL_ERROR(e_)
	} catch (mrrocpp::lib::exception::fatal_error & e_) {
		// Standard error handling.
		HANDLE_EDP_FATAL_ERROR(e_)
	} catch (mrrocpp::lib::exception::system_error & e_) {
		// Standard error handling.
		HANDLE_EDP_SYSTEM_ERROR(e_)
	} catch (...) {
		HANDLE_EDP_UNKNOWN_ERROR()
	}
}

void effector::get_controller_state(lib::c_buffer &instruction)
{

	if (robot_test_mode) {
		return;
	}

	try {
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
		get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

		{
			boost::mutex::scoped_lock lock(effector_mutex);

			// Ustawienie poprzedniej wartosci zadanej na obecnie odczytane polozenie walow silnikow
			for (int i = 0; i < number_of_servos; i++) {
				servo_current_motor_pos[i] = desired_motor_pos_new[i] = desired_motor_pos_old[i] = current_motor_pos[i];
				desired_joints[i] = current_joints[i];
			}
		}
	} catch (mrrocpp::lib::exception::non_fatal_error & e_) {
		// Standard error handling.
		HANDLE_EDP_NON_FATAL_ERROR(e_)
	} catch (mrrocpp::lib::exception::fatal_error & e_) {
		// Standard error handling.
		HANDLE_EDP_FATAL_ERROR(e_)
	} catch (mrrocpp::lib::exception::system_error & e_) {
		// Standard error handling.
		HANDLE_EDP_SYSTEM_ERROR(e_)
	} catch (...) {
		HANDLE_EDP_UNKNOWN_ERROR()
	}

}

/*--------------------------------------------------------------------------*/
void effector::move_arm(const lib::c_buffer &instruction)
{
	try {
		msg->message("move_arm");

		std::stringstream ss(std::stringstream::in | std::stringstream::out);

		switch (ecp_edp_cbuffer.variant)
		{
			case lib::shead::POSE: {
				msg->message("POSE");
			}
				break;

			case lib::shead::QUICKSTOP: {
				msg->message("QUICKSTOP");
			}
				break;

			case lib::shead::CLEAR_FAULT: {
				msg->message("CLEAR_FAULT");
			}
				break;
			case lib::shead::SOLIDIFICATION: {
				lib::shead::SOLIDIFICATION_ACTIVATION head_solidification;

				memcpy(&head_solidification, &(ecp_edp_cbuffer.head_solidification), sizeof(head_solidification));
				ss << "SOLIDIFICATION: " << head_solidification;
				msg->message(ss.str().c_str());

				// previously computed parameters send to epos2 controllers

				// start the trajectory execution

			}
				break;
			case lib::shead::VACUUM: {
				lib::shead::VACUUM_ACTIVATION vacuum_activation;

				memcpy(&vacuum_activation, &(ecp_edp_cbuffer.vacuum_activation), sizeof(vacuum_activation));
				ss << "VACUUM: " << vacuum_activation;
				msg->message(ss.str().c_str());
			}
				break;
			default:
				break;

		}
	} catch (mrrocpp::lib::exception::non_fatal_error & e_) {
		// Standard error handling.
		HANDLE_EDP_NON_FATAL_ERROR(e_)
	} catch (mrrocpp::lib::exception::fatal_error & e_) {
		// Standard error handling.
		HANDLE_EDP_FATAL_ERROR(e_)
	} catch (mrrocpp::lib::exception::system_error & e_) {
		// Standard error handling.
		HANDLE_EDP_SYSTEM_ERROR(e_)
	} catch (...) {
		HANDLE_EDP_UNKNOWN_ERROR()
	}
}

/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{
	try {
		//lib::JointArray desired_joints_tmp(lib::MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
		//	printf(" GET ARM\n");
		//	flushall();
		static int licznikaaa = (-11);

		std::stringstream ss(std::stringstream::in | std::stringstream::out);
		ss << "get_arm_position: " << licznikaaa;
		msg->message(ss.str().c_str());
		//	printf("%s\n", ss.str().c_str());

		reply.servo_step = step_counter;
		edp_ecp_rbuffer.shead_reply.soldification_state = lib::shead::SOLDIFICATION_STATE_INTERMEDIATE;
		edp_ecp_rbuffer.shead_reply.vacuum_state = lib::shead::VACUUM_STATE_OFF;
		edp_ecp_rbuffer.shead_reply.contacts[1] = true;

	} catch (mrrocpp::lib::exception::non_fatal_error & e_) {
		// Standard error handling.
		HANDLE_EDP_NON_FATAL_ERROR(e_)
	} catch (mrrocpp::lib::exception::fatal_error & e_) {
		// Standard error handling.
		HANDLE_EDP_FATAL_ERROR(e_)
	} catch (mrrocpp::lib::exception::system_error & e_) {
		// Standard error handling.
		HANDLE_EDP_SYSTEM_ERROR(e_)
	} catch (...) {
		HANDLE_EDP_UNKNOWN_ERROR()
	}
}
/*--------------------------------------------------------------------------*/

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::shead::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

/*--------------------------------------------------------------------------*/
void effector::create_threads()
{
	rb_obj = (boost::shared_ptr <common::reader_buffer>) new common::reader_buffer(*this);
	vis_obj = (boost::shared_ptr <common::vis_server>) new common::vis_server(*this);
}

void effector::instruction_deserialization()
{
	memcpy(&ecp_edp_cbuffer, instruction.serialized_command, sizeof(ecp_edp_cbuffer));
}

void effector::reply_serialization(void)
{
	assert(sizeof(reply.serialized_reply) >= sizeof(edp_ecp_rbuffer));
	memcpy(reply.serialized_reply, &edp_ecp_rbuffer, sizeof(edp_ecp_rbuffer));
}

} // namespace shead
} // namespace edp
} // namespace mrrocpp

