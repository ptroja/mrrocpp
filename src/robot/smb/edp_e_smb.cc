#include <cstdio>
#include <iostream>
#include <bitset>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "edp_e_smb.h"

#include "base/edp/reader.h"
// Kinematyki.
#include "robot/smb/kinematic_model_smb.h"
#include "base/edp/manip_trans_t.h"
#include "base/edp/vis_server.h"

#include "base/lib/exception.h"
using namespace mrrocpp::lib;
using namespace mrrocpp::lib::exception;
using namespace std;

#define FESTO_C1_GROUP 1
#define FESTO_C1_BIT (1<<0)

#define FESTO_C2_GROUP 2
#define FESTO_C2_BIT (1<<1)

#define FESTO_C3_GROUP 2
#define FESTO_C3_BIT (1<<0)

#define FESTO_CY11_GROUP 1
#define FESTO_CY11_BIT (1<<3)

#define FESTO_CY12_GROUP 1
#define FESTO_CY12_BIT (1<<2)

#define FESTO_CY21_GROUP 1
#define FESTO_CY21_BIT (1<<5)

#define FESTO_CY22_GROUP 1
#define FESTO_CY22_BIT (1<<4)

#define FESTO_CY31_GROUP 1
#define FESTO_CY31_BIT (1<<7)

#define FESTO_CY32_GROUP 1
#define FESTO_CY32_BIT (1<<6)

#define FESTO_CH1_GROUP 2
#define FESTO_CH1_BIT (1<<5)

#define FESTO_CH2_GROUP 2
#define FESTO_CH2_BIT (1<<3)

#define FESTO_CH3_GROUP 2
#define FESTO_CH3_BIT (1<<4)

#define FESTO_A1_GROUP 1
#define FESTO_A1_BIT (1<<1)

#define FESTO_A2_GROUP 2
#define FESTO_A2_BIT (1<<2)

#define FESTO_A3_GROUP 2
#define FESTO_A3_BIT (1<<7)

#define FESTO_H1_GROUP 2
#define FESTO_H1_BIT (1<<6)

const uint8_t nodeId = 10;

namespace mrrocpp {
namespace edp {
namespace smb {

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	motor_driven_effector::single_thread_master_order(nm_task, nm_tryb);
}

void effector::get_controller_state(lib::c_buffer &instruction)
{

	if (robot_test_mode) {
		controller_state_edp_buf.is_synchronised = false;
	}
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
}

// Konstruktor.
effector::effector(common::shell &_shell, lib::robot_name_t l_robot_name) :
		motor_driven_effector(_shell, l_robot_name)
{

	number_of_servos = lib::smb::NUM_OF_SERVOS;
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();

	if (!robot_test_mode) {
		// Create gateway object.
		if (this->config.exists("can_iface")) {
			gateway =
					(boost::shared_ptr <canopen::gateway>) new canopen::gateway_socketcan(config.value <std::string>("can_iface"));
		} else {
			gateway = (boost::shared_ptr <canopen::gateway>) new canopen::gateway_epos_usb();
		}

		// Connect to the gateway.
		gateway->open();

		// Create epos objects according to CAN ID-mapping.
		epos_di_node = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 8);

		// TODO - odczytac current_state
		// current_state = next_state =

	} else {
		current_state = next_state = lib::smb::ALL_UP;

		for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {

			edp_ecp_rbuffer.multi_leg_reply.leg[i].is_up = true;
			edp_ecp_rbuffer.multi_leg_reply.leg[i].is_down = false;

		}
	}

}

void effector::synchronise(void)
{
	if (robot_test_mode) {
		controller_state_edp_buf.is_synchronised = true;
		return;
	}
}

/*--------------------------------------------------------------------------*/
void effector::move_arm(const lib::c_buffer &instruction)
{
	msg->message("move_arm");

	std::stringstream ss(std::stringstream::in | std::stringstream::out);

	// the previous next_state becomes currrent_state
	current_state = next_state;

	switch (ecp_edp_cbuffer.variant)
	{
		case lib::smb::POSE: {
			msg->message("POSE");

			ss << ecp_edp_cbuffer.motor_pos[1];

			msg->message(ss.str().c_str());

		}
			break;
		case lib::smb::QUICKSTOP: {
			msg->message("QUICKSTOP");
		}
			break;

		case lib::smb::CLEAR_FAULT: {
			msg->message("CLEAR_FAULT");
		}
			break;

		case lib::smb::FESTO: {
			festo_command();
		}
			break;
		default:
			break;

	}

}
/*--------------------------------------------------------------------------*/

void effector::festo_command()
{
	std::stringstream ss(std::stringstream::in | std::stringstream::out);

	msg->message("FESTO");
	lib::smb::festo_command_td festo_command;

	memcpy(&festo_command, &(ecp_edp_cbuffer.festo_command), sizeof(festo_command));

	if (robot_test_mode) {
		ss << festo_command.leg[2];

		msg->message(ss.str().c_str());
	}

	// determine next_state by counting numebr of legs to be up
	int number_of_legs_up = 0;
	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		if (festo_command.leg[i] == lib::smb::UP) {
			number_of_legs_up++;
		}

	}

	switch (number_of_legs_up)
	{
		case 0:
			next_state = lib::smb::ALL_DOWN;
			break;
		case 1:
			next_state = lib::smb::ONE_UP_TWO_DOWN;
			break;
		case 2:
			next_state = lib::smb::TWO_UP_ONE_DOWN;
			break;
		case 3:
			next_state = lib::smb::ALL_UP;
			break;
		default:
			break;

	}

	// checks if the next_state is valid taking into account current_state
	// and prepares detailed command for festo hardware

	switch (next_state)
	{
		case lib::smb::ALL_DOWN:
			festo_command_all_down(festo_command);
			break;
		case lib::smb::ONE_UP_TWO_DOWN:
			festo_command_one_up_two_down(festo_command);
			break;
		case lib::smb::TWO_UP_ONE_DOWN:
			festo_command_two_up_one_down(festo_command);
			break;
		case lib::smb::ALL_UP:
			festo_command_all_up(festo_command);
			break;
		default:
			break;

	}

}

void effector::festo_command_all_down(lib::smb::festo_command_td& festo_command)
{
	switch (current_state)
	{
		case lib::smb::ALL_DOWN:
			throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
			break;
		case lib::smb::ONE_UP_TWO_DOWN:
			festo_test_mode_set_reply(festo_command);
			break;
		case lib::smb::TWO_UP_ONE_DOWN:
			festo_test_mode_set_reply(festo_command);
			break;
		case lib::smb::ALL_UP:
			festo_test_mode_set_reply(festo_command);
			break;
		default:
			break;

	}
}

void effector::festo_command_one_up_two_down(lib::smb::festo_command_td& festo_command)
{
	throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
}

void effector::festo_command_two_up_one_down(lib::smb::festo_command_td& festo_command)
{
	switch (current_state)
	{
		case lib::smb::ALL_DOWN: {
			festo_test_mode_set_reply(festo_command);
		}

			break;
		case lib::smb::ONE_UP_TWO_DOWN:
		case lib::smb::TWO_UP_ONE_DOWN:
		case lib::smb::ALL_UP:
			throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
			break;
		default:
			break;

	}
}

void effector::festo_command_all_up(lib::smb::festo_command_td& festo_command)
{
	switch (current_state)
	{
		case lib::smb::ALL_DOWN: {
			festo_test_mode_set_reply(festo_command);
		}

			break;
		case lib::smb::ONE_UP_TWO_DOWN:
		case lib::smb::TWO_UP_ONE_DOWN:
		case lib::smb::ALL_UP:
			throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
			break;
		default:
			break;

	}
}

void effector::festo_test_mode_set_reply(lib::smb::festo_command_td& festo_command)
{
	if (robot_test_mode) {
		for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
			if (festo_command.leg[i] == lib::smb::UP) {
				edp_ecp_rbuffer.multi_leg_reply.leg[i].is_up = true;
				edp_ecp_rbuffer.multi_leg_reply.leg[i].is_down = false;
			} else {
				edp_ecp_rbuffer.multi_leg_reply.leg[i].is_up = false;
				edp_ecp_rbuffer.multi_leg_reply.leg[i].is_down = true;
			}

		}
	}
}

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{
	//lib::JointArray desired_joints_tmp(lib::MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	//	printf(" GET ARM\n");
	//	flushall();
	static int licznikaaa = (-11);
	if (robot_test_mode) {
		std::stringstream ss(std::stringstream::in | std::stringstream::out);
		ss << "get_arm_position: " << licznikaaa;
		msg->message(ss.str().c_str());
		//	printf("%s\n", ss.str().c_str());

		edp_ecp_rbuffer.epos_controller[1].position = licznikaaa;

		if (licznikaaa < 10) {
			for (int i = 0; i < number_of_servos; i++) {
				edp_ecp_rbuffer.epos_controller[i].motion_in_progress = true;
			}

		} else {
			for (int i = 0; i < number_of_servos; i++) {
				edp_ecp_rbuffer.epos_controller[i].motion_in_progress = false;
			}

		}

		licznikaaa++;
	} else {
		std::bitset <16> epos_digits = epos_di_node->readDInput();
		for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
			edp_ecp_rbuffer.multi_leg_reply.leg[i].is_down = epos_digits[2 * i + 10];
			edp_ecp_rbuffer.multi_leg_reply.leg[i].is_up = epos_digits[2 * i + 11];
		}
		//std::cout << "epos digital inputs = " << epos_digits << std::endl;
	}

	reply.servo_step = step_counter;
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

void effector::create_threads()
{
	rb_obj = (boost::shared_ptr <common::reader_buffer>) new common::reader_buffer(*this);
	vis_obj = (boost::shared_ptr <common::vis_server>) new common::vis_server(*this);
}

void effector::instruction_deserialization()
{
	memcpy(&ecp_edp_cbuffer, instruction.arm.serialized_command, sizeof(ecp_edp_cbuffer));
}

void effector::reply_serialization(void)
{
	memcpy(reply.arm.serialized_reply, &edp_ecp_rbuffer, sizeof(edp_ecp_rbuffer));
	assert(sizeof(reply.arm.serialized_reply) >= sizeof(edp_ecp_rbuffer));
}

} // namespace smb
} // namespace edp
} // namespace mrrocpp

