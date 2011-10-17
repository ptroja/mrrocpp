#include <cstdio>
#include <iostream>
#include <bitset>
#include <boost/foreach.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "edp_e_smb.h"
#include "festo_and_inputs.h"

#include "base/edp/reader.h"
// Kinematyki.
#include "robot/smb/kinematic_model_smb.h"
#include "base/edp/manip_trans_t.h"
#include "base/edp/vis_server.h"

#include "exceptions.h"

using namespace std;

namespace mrrocpp {
namespace edp {
namespace smb {

/*
 TODO: Set propper values of A and V.
 const uint32_t effector::Vdefault[lib::smb::NUM_OF_SERVOS] = { 100UL, 100UL };
 const uint32_t effector::Adefault[lib::smb::NUM_OF_SERVOS] = { 1000UL, 1000UL };
 const uint32_t effector::Ddefault[lib::smb::NUM_OF_SERVOS] = { 1000UL, 1000UL };
 */

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
			gateway
					= (boost::shared_ptr <canopen::gateway>) new canopen::gateway_socketcan(config.value <std::string> ("can_iface"));
		} else {
			gateway = (boost::shared_ptr <canopen::gateway>) new canopen::gateway_epos_usb();
		}

		// Connect to the gateway.
		gateway->open();

		// Create epos objects according to CAN ID-mapping.
		epos_di_node = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 8);
		pkm_rotation_node = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 9);

		// Collect axes into common array container.
		axes[0] = &(*epos_di_node);
		axesNames[0] = "legs";
		axes[1] = &(*pkm_rotation_node);
		axesNames[1] = "pkm";

		// Create festo node.
		cpv10 = (boost::shared_ptr <festo::cpv>) new festo::cpv(*gateway, 10);
	} else {

	}

}

void effector::reset_variables()
{
	// Zero all variables related to motor positions.
	for (int i = 0; i < number_of_servos; ++i) {
		current_motor_pos[i] = 0;
	}
	desired_motor_pos_old = current_motor_pos;
	desired_motor_pos_new = current_motor_pos;

	// Compute current motor positions on the base of zeroed motors.
	get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
	desired_joints = current_joints;
}

void effector::synchronise(void)
{
	if (robot_test_mode) {
		controller_state_edp_buf.is_synchronised = true;
		return;
	}
#if 0
	// Two-step synchronization of the motor rotating the whole PKM.
	// Step1: Potentiometer.
	// Get current potentiometer readings.
	int pot = pkm_rotation_node->readAnalogInput1();

	// Set coefficients.
	const double p1 = -0.0078258336;
	const double p2 = 174.7796278191;
	const double p3 = -507883.404901415;

	// Compute desired position.
	int position = -(pot * pot * p1 + pot * p2 + p3) - 120000;
	cout << "Retrieved potentiometer reading: " << pot << "\nComputed pose: " << position << endl;

	// Move to the relative position.
	pkm_rotation_node->moveRelative(position);
	while (!pkm_rotation_node->isTargetReached())
		usleep(200000);

	// Step2: Homing.
	// Activate homing mode.
	pkm_rotation_node->doHoming(maxon::epos::HM_INDEX_NEGATIVE_SPEED, 0);


	// Compute joints positions in the home position
	get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
#endif

	// Now the robot is synchronised
	controller_state_edp_buf.is_synchronised = true;

}

lib::smb::ALL_LEGS_VARIANT effector::current_legs_state(void)
{
	return fai->current_legs_state;
}

lib::smb::ALL_LEGS_VARIANT effector::next_legs_state(void)
{
	return fai->next_legs_state;
}

/*--------------------------------------------------------------------------*/
void effector::move_arm(const lib::c_buffer &instruction)
{
	try {
		msg->message("move_arm");

		switch (ecp_edp_cbuffer.variant)
		{
			case lib::smb::POSE: {
				msg->message("POSE");
				// Control the two SMB rotational motors.
				rotational_motors_command();
			}
				break;
			case lib::smb::QUICKSTOP: {
				msg->message("QUICKSTOP");
			}
				break;

			case lib::smb::CLEAR_FAULT: {
				BOOST_FOREACH(maxon::epos * node, axes)
				{
					node->printEPOSstate();

					// Check if in a FAULT state
					if (node->checkEPOSstate() == 11) {
						maxon::UNSIGNED8 errNum = node->readNumberOfErrors();
						cerr << "readNumberOfErrors() = " << (int) errNum << endl;
						// Print list of errors.
						for (maxon::UNSIGNED8 i = 1; i <= errNum; ++i) {
							maxon::UNSIGNED32 errCode = node->readErrorHistory(i);
							cerr << node->ErrorCodeMessage(errCode) << endl;
						}
						// Clear errors.
						if (errNum > 0) {
							node->clearNumberOfErrors();
						}
						node->changeEPOSstate(maxon::epos::FAULT_RESET);
					}
					// Reset node.
					//node->reset();
				}
				// Internal position counters need not be updated
				return;
			}
			case lib::smb::FESTO: {
				if (is_base_positioned_to_move_legs) {
					fai->festo_command();
				}
			}
				break;
			default:
				break;

		}
	} catch (mrrocpp::lib::exception::mrrocpp_non_fatal_error & e_) {
		// Standard error handling.
		HANDLE_MRROCPP_ERROR(e_)
	}

}

void effector::rotational_motors_command()
{
	/*	std::stringstream ss(std::stringstream::in | std::stringstream::out);
	 ss << ecp_edp_cbuffer.motor_pos[1];
	 msg->message(ss.str().c_str());
	 ss << ecp_edp_cbuffer.pose_specification;
	 msg->message(ss.str().c_str());*/

	if (current_legs_state() != lib::smb::TWO_UP_ONE_DOWN) {
		// The TWO_UP_ONE_DOWN is the only state in which control of both motors (legs and SPKM rotations) is possible.
		// In other states control of the motor rotating the legs (lower SMB motor) is prohibited!

		// Check the difference between current and desired values.
		// Check motors.
		if ((ecp_edp_cbuffer.pose_specification == lib::smb::MOTOR) && (current_motor_pos[0]
				!= ecp_edp_cbuffer.motor_pos[0]))
			BOOST_THROW_EXCEPTION(mrrocpp::edp::smb::nfe_clamps_rotation_prohibited_in_given_state()<<current_state(current_legs_state()));
		// Check joints.
		else if ((ecp_edp_cbuffer.pose_specification == lib::smb::JOINT) && (current_joints[0]
				!= ecp_edp_cbuffer.motor_pos[0]))
			BOOST_THROW_EXCEPTION(mrrocpp::edp::smb::nfe_clamps_rotation_prohibited_in_given_state()<<current_state(current_legs_state()));
	}

	// TODO: remove this line!
	ecp_edp_cbuffer.pose_specification = lib::smb::MOTOR;

	// Interpret command according to the pose specification.
	switch (ecp_edp_cbuffer.pose_specification)
	{
		case lib::smb::MOTOR:
			// Copy data directly from buffer
			for (int i = 0; i < number_of_servos; ++i) {
				desired_motor_pos_new[i] = ecp_edp_cbuffer.motor_pos[i];
				cout << "MOTOR[ " << i << "]: " << desired_motor_pos_new[i] << endl;
			}
			// Check the desired motor (only motors!) values if they are absolute.
			get_current_kinematic_model()->check_motor_position(desired_motor_pos_new);
			// Transform desired motors to joints.
			get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, desired_joints);
			break;
		default:
			// Throw non-fatal error - invalid pose specification.
			BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_pose_specification());
			break;
	} //: switch (ecp_edp_cbuffer.pose_specification)


	// TODO: remove this line!
	ecp_edp_cbuffer.motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;

	// Perform motion depending on its type.
	// Note: at this point we assume, that desired_motor_pos_new holds a validated data.
	switch (ecp_edp_cbuffer.motion_variant)
	{
		case lib::epos::NON_SYNC_TRAPEZOIDAL:
			// Execute command.
			for (size_t i = 0; i < axes.size(); ++i) {
				if (is_synchronised()) {
					cout << "MOTOR: moveAbsolute[" << i << "] ( " << desired_motor_pos_new[i] << ")"
							<< endl;
					if (!robot_test_mode) {
/*						axes[i]->writeProfileVelocity(Vdefault[i]);
						axes[i]->writeProfileAcceleration(Adefault[i]);
						axes[i]->writeProfileDeceleration(Ddefault[i]);*/
						axes[i]->moveAbsolute(desired_motor_pos_new[i]);
					} else {
						current_joints[i] = desired_joints[i];
						current_motor_pos[i] = desired_motor_pos_new[i];
					}
				} else {
					cout << "MOTOR: moveRelative[" << i << "] ( " << desired_motor_pos_new[i] << ")"
							<< endl;
					if (!robot_test_mode) {
/*						axes[i]->writeProfileVelocity(Vdefault[i]);
						axes[i]->writeProfileAcceleration(Adefault[i]);
						axes[i]->writeProfileDeceleration(Ddefault[i]);*/
						axes[i]->moveRelative(desired_motor_pos_new[i]);
					} else {
						current_joints[i] += desired_joints[i];
						current_motor_pos[i] += desired_motor_pos_new[i];
					}
				}
			}
			break;
		default:
			// Throw non-fatal error - motion type not supported.
			BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_motion_type());
			break;
	}//: switch (ecp_edp_cbuffer.motion_variant)

}

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{
	/*	std::stringstream ss(std::stringstream::in | std::stringstream::out);
	 ss << instruction.get_arm_type;
	 msg->message(ss.str().c_str());*/

	// TODO: remove this line!
	instruction.get_arm_type = lib::MOTOR;

	// Check motion type.
	switch (instruction.get_arm_type)
	{
		case lib::MOTOR:
			msg->message("EDP get_arm_position MOTOR");
			for (size_t i = 0; i < number_of_servos; ++i) {
				if (robot_test_mode) {
					edp_ecp_rbuffer.epos_controller[i].position = current_motor_pos[i];
					edp_ecp_rbuffer.epos_controller[i].current = 0;
					edp_ecp_rbuffer.epos_controller[i].motion_in_progress = false;
				} else {
					current_motor_pos[i] = axes[i]->readActualPosition();
					edp_ecp_rbuffer.epos_controller[i].position = current_motor_pos[i];
					edp_ecp_rbuffer.epos_controller[i].current = axes[i]->readActualCurrent();
					edp_ecp_rbuffer.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
				}
			}
			break;
		case lib::JOINT:
			msg->message("EDP get_arm_position JOINT");

			// Read actual values from hardware
			/*			if (!robot_test_mode) {
			 for (size_t i = 0; i < axes.size(); ++i) {
			 current_motor_pos[i] = axes[i]->readActualPosition();
			 }
			 }*/

			// Calculate current joint values.
			get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

			// Copy values to buffer.
			for (int i = 0; i < number_of_servos; ++i) {
				edp_ecp_rbuffer.epos_controller[i].position = current_joints[i];
			}
			break;
		default:
			break;

	}

	// SMB clamps.

	fai->create_reply();

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
	fai = new festo_and_inputs(*this);
	rb_obj = (boost::shared_ptr <common::reader_buffer>) new common::reader_buffer(*this);
	vis_obj = (boost::shared_ptr <common::vis_server>) new common::vis_server(*this);

	fai->initiate();

	// do poprawy
	is_base_positioned_to_move_legs = true;

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

