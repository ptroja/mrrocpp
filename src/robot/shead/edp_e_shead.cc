#include <cstdio>
#include <boost/static_assert.hpp>

#include "base/lib/debug.hpp"

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

#include "robot/shead/edp_e_shead.h"
#include "base/edp/reader.h"

#include "robot/shead/kinematic_model_shead.h"

#include "base/lib/exception.h"

#include "robot/canopen/gateway_epos_usb.h"
#include "robot/canopen/gateway_socketcan.h"
#include "robot/maxon/epos.h"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace shead {

#include "base/lib/debug.hpp"

const uint32_t effector::Vdefault = 300UL;
const uint32_t effector::Adefault = 300UL;
const uint32_t effector::Ddefault = 300UL;

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	motor_driven_effector::single_thread_master_order(nm_task, nm_tryb);
}

// Konstruktor.
effector::effector(common::shell &_shell, lib::robot_name_t l_robot_name) :
		motor_driven_effector(_shell, l_robot_name, instruction, reply)
{
	DEBUG_METHOD;

	// Set number of servos.
	number_of_servos = lib::shead::NUM_OF_SERVOS;

	// Create kinematics.
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
		epos_node = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 7, "head");
	}
}

void effector::synchronise(void)
{
	DEBUG_METHOD;

	try {
		if (robot_test_mode) {
			controller_state_edp_buf.is_synchronised = true;
			return;
		}

		// Check state of the robot.
		if (controller_state_edp_buf.robot_in_fault_state)
			BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::fe_robot_in_fault_state());

		// Common synchronization sequence
		epos_node->setOperationMode(maxon::epos::OMD_HOMING_MODE);
		epos_node->reset();
		epos_node->startHoming();
		epos_node->monitorHomingStatus();

		controller_state_edp_buf.is_synchronised = true;

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

void effector::check_controller_state()
{
	DEBUG_METHOD;

	if (robot_test_mode) {
		// In test mode robot is always synchronized.
		controller_state_edp_buf.is_synchronised = true;
		return;
	}

	// Try to get state of each axis
	bool powerOn = false;
	bool enabled = false;

	maxon::UNSIGNED16 statusWord = 0x0000;

	try {
		// Get current epos statusword.

		statusWord = epos_node->getStatusWord();

		maxon::epos::actual_state_t state = epos_node->status2state(statusWord);

		if (state != maxon::epos::OPERATION_ENABLE) {
			// Print state.
			epos_node->printState();
			// Check if in the FAULT state
			if (state == maxon::epos::FAULT) {
				// Read number of errors
				int errNum = epos_node->getNumberOfErrors();
				for (int j = 1; j <= errNum; ++j) {
					// Get the detailed error
					uint32_t errCode = epos_node->getErrorHistory(j);
					// Send message to SR.
					msg->message(mrrocpp::lib::FATAL_ERROR, epos_node->ErrorCodeMessage(errCode));
				}
			} else if (state == maxon::epos::SWITCH_ON_DISABLED) {
				// Send message to SR.
				msg->message(mrrocpp::lib::FATAL_ERROR, "EPOS controller in DISABLED state");
			} //: if fault || disabled
		} else {
			// EPOS in enabled state.
			enabled = true;
		}
		// At this point we are sure that axis is working, thus powered.
		powerOn = true;
	} catch (...) {
		// Probably the axis is not powered on, do nothing.
	}

	// Robot is synchronized the one axis is referenced.
	controller_state_edp_buf.is_synchronised = epos_node->isReferenced(statusWord);
	// Check whether robot is powered.
	controller_state_edp_buf.is_power_on = powerOn;
	// Check fault state.
	controller_state_edp_buf.robot_in_fault_state = (!enabled);
}

void effector::get_controller_state(lib::c_buffer &instruction_)
{
	DEBUG_METHOD;

	try {
		// False is the initial value
		controller_state_edp_buf.is_synchronised = false;
		controller_state_edp_buf.is_power_on = false;
		controller_state_edp_buf.robot_in_fault_state = false;

		// Check controller state.
		check_controller_state();

		// Copy data to reply buffer
		reply.controller_state = controller_state_edp_buf;

		// Initiate motor positions.

		// If this is not a test mode and robot is synchronized.
		if (!robot_test_mode && is_synchronised()) {
			// Get actual motor positions.
			current_motor_pos[0] = epos_node->getActualPosition();
		} else {
			// Zero all motor positions.
			current_motor_pos[0] = 0;
		}
#if(DEBUG_MOTORS)
		std::cout << "current_motor_pos: " << current_motor_pos.transpose() << std::endl;
#endif

		// Compute current motor positions on the base of current motors.
		get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

#if(DEBUG_JOINTS)
		std::cout << "current_joints: " << current_joints.transpose() << std::endl;
#endif

		// Lock data structure during update
		{
			boost::mutex::scoped_lock lock(effector_mutex);

			// Initialize internal data.
			servo_current_motor_pos = current_motor_pos;
			desired_motor_pos_old = current_motor_pos;
			desired_motor_pos_new = current_motor_pos;
			desired_joints = current_joints;
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

void effector::parse_motor_command()
{
	DEBUG_METHOD;

	// Check state of the robot.
	if (controller_state_edp_buf.robot_in_fault_state)
		BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::fe_robot_in_fault_state());

	// Interpret command according to the pose specification.
	switch (instruction.shead.set_pose_specification)
	{
		case lib::shead::MOTOR:
			DEBUG_COMMAND("MOTOR");
			// Copy data directly from buffer.
			desired_motor_pos_new[0] = instruction.shead.motor_pos[0];

			// Check the desired motor (only motors!) values if they are absolute.
			get_current_kinematic_model()->check_motor_position(desired_motor_pos_new);
			// Transform desired motors to joints.
			get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, desired_joints);
			break;
		case lib::shead::JOINT:
			DEBUG_COMMAND("JOINT");
			// Copy data directly from buffer.
			desired_joints[0] = instruction.shead.joint_pos[0];

			if (is_synchronised()) {
				// Transform desired joint to motors (and check motors/joints values).
				get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new, desired_joints);
				// Postcondition - check whether the desired motor position is valid.
				get_current_kinematic_model()->check_motor_position(desired_motor_pos_new);
			} else {
				// Throw non-fatal error - this mode requires synchronization.
				BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_robot_unsynchronized());
			}
			break;
		default:
			// Throw non-fatal error - invalid pose specification.
			BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_pose_specification());
			break;
	} //: switch (instruction.shead.set_pose_specification)
}

void effector::execute_motor_motion()
{
	DEBUG_METHOD;

	// Execute command.
	if (is_synchronised()) {
		// Robot is synchronized.
		if (!robot_test_mode) {
			// Set velocity and acceleration values.
			epos_node->setProfileVelocity(Vdefault);
			epos_node->setProfileAcceleration(Adefault);
			epos_node->setProfileDeceleration(Ddefault);
			epos_node->moveAbsolute(desired_motor_pos_new[0]);
		} else {
			// Virtually "move" to desired absolute position.
			current_motor_pos[0] = desired_motor_pos_new[0];
		}
	} else {
		// Robot unsynchronized.
		if (!robot_test_mode) {
			// Set velocity and acceleration values.
			epos_node->setProfileVelocity(Vdefault);
			epos_node->setProfileAcceleration(Adefault);
			epos_node->setProfileDeceleration(Ddefault);
			epos_node->moveRelative(desired_motor_pos_new[0]);
		} else {
			// Virtually "move" to desired relative position.
			current_motor_pos[0] += desired_motor_pos_new[0];
		}
	} //: is_synchronised
}

/*--------------------------------------------------------------------------*/
void effector::move_arm(const lib::c_buffer &instruction_)
{
	DEBUG_METHOD;

	try {

		switch (instruction.shead.variant)
		{
			case lib::shead::POSE:
				DEBUG_COMMAND("POSE");
				// Control the two SMB rotational motors.
				// Parse command.
				parse_motor_command();
				// Execute motion.
				execute_motor_motion();
				break;
			case lib::shead::QUICKSTOP:
				DEBUG_COMMAND("QUICKSTOP");
				if (!robot_test_mode) {
					// Brake with Quickstop command
					epos_node->setState(maxon::epos::QUICKSTOP);

					// Switch back to ENABLE state.
					epos_node->reset(); //(maxon::epos::ENABLE_OPERATION);
				}
				break;
			case lib::shead::CLEAR_FAULT:
				DEBUG_COMMAND("CLEAR_FAULT");
				if (!robot_test_mode) {
					epos_node->clearFault();
				}
				break;
			case lib::shead::SOLIDIFICATION:
				DEBUG_COMMAND("SOLIDIFICATION");

				// Check state of the robot.
				if (controller_state_edp_buf.robot_in_fault_state)
					BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::fe_robot_in_fault_state());

				switch (instruction.shead.head_solidification)
				{
					case lib::shead::SOLIDIFICATION_STATE_ON:
						if (!robot_test_mode) {
							// Get current output state
							maxon::epos::digital_outputs_t outputs = epos_node->getCommandedDigitalOutputs();

							// Enable General purpose OutB and OutC
							outputs[1] = true;
							outputs[2] = true;

							// Set new state
							epos_node->setDigitalOutputs(outputs);
						} else {
							virtual_state.solidification_state = lib::shead::SOLIDIFICATION_STATE_ON;
						}
						break;
					case lib::shead::SOLIDIFICATION_STATE_OFF:
						if (!robot_test_mode) {
							// Get current output state
							maxon::epos::digital_outputs_t outputs = epos_node->getCommandedDigitalOutputs();

							// Disable General purpose OutB and OutC
							outputs[1] = false;
							outputs[2] = false;

							// Set new state
							epos_node->setDigitalOutputs(outputs);
						} else {
							virtual_state.solidification_state = lib::shead::SOLIDIFICATION_STATE_OFF;
						}
						break;
					default:
						// Throw non-fatal error - command not supported.
						BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_command());
						break;
				}
				break;
			case lib::shead::VACUUM:
				DEBUG_COMMAND("VACUUM");

				// Check state of the robot.
				if (controller_state_edp_buf.robot_in_fault_state)
					BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::fe_robot_in_fault_state());

				switch (instruction.shead.vacuum_activation)
				{
					case lib::shead::VACUUM_ON:
						if (!robot_test_mode) {
							// Get current output state
							maxon::epos::digital_outputs_t outputs = epos_node->getCommandedDigitalOutputs();

							// Enable General purpose OutD
							outputs[3] = true;

							// Set new state
							epos_node->setDigitalOutputs(outputs);
						} else {
							virtual_state.vacuum_state = lib::shead::VACUUM_STATE_ON;
						}
						break;
					case lib::shead::VACUUM_OFF:
						if (!robot_test_mode) {
							// Get current output state
							maxon::epos::digital_outputs_t outputs = epos_node->getCommandedDigitalOutputs();

							// Disable General purpose OutD
							outputs[3] = false;

							// Set new state
							epos_node->setDigitalOutputs(outputs);
						} else {
							virtual_state.vacuum_state = lib::shead::VACUUM_STATE_OFF;
						}
						break;
					default:
						// Throw non-fatal error - command not supported.
						BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_command());
						break;
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

void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction_)
{
	DEBUG_METHOD;

	try {
		// Check controller state.
		check_controller_state();

		// Handle only GET and SET_GET instructions.
		if (instruction.instruction_type != lib::SET) {

			if (robot_test_mode) {
				// Copy data from virtual state
				reply.shead.shead_reply.solidification_state = virtual_state.solidification_state;
				reply.shead.shead_reply.vacuum_state = virtual_state.vacuum_state;
			} else {
				// Ask about current solidification control pins state
				reply.shead.shead_reply.solidification_state =
						(epos_node->getCommandedDigitalOutputs()[1] && epos_node->getCommandedDigitalOutputs()[2]) ? lib::shead::SOLIDIFICATION_STATE_ON : lib::shead::SOLIDIFICATION_STATE_OFF;

				// Ask about current vacuum control pins state
				reply.shead.shead_reply.vacuum_state =
						(epos_node->getCommandedDigitalOutputs()[3]) ? lib::shead::VACUUM_STATE_ON : lib::shead::VACUUM_STATE_OFF;
			}

			switch (instruction.shead.get_pose_specification)
			{
				case lib::shead::MOTOR:
					DEBUG_COMMAND("MOTOR");
					if (!robot_test_mode) {
						// Update current position.
						current_motor_pos[0] = epos_node->getActualPosition();
						// Copy values to buffer.
						reply.shead.epos_controller.position = current_motor_pos[0];
						reply.shead.epos_controller.current = epos_node->getActualCurrent();
						reply.shead.epos_controller.motion_in_progress = !epos_node->isTargetReached();
					} else {
						// Copy values to buffer.
						reply.shead.epos_controller.position = current_motor_pos[0];
						reply.shead.epos_controller.current = 0;
						reply.shead.epos_controller.motion_in_progress = false;
					}
					break;
				case lib::shead::JOINT:
					DEBUG_COMMAND("JOINT");
					if (!robot_test_mode) {
						// Update current position.
						current_motor_pos[0] = epos_node->getActualPosition();
						// Copy values to buffer.
						reply.shead.epos_controller.current = epos_node->getActualCurrent();
						reply.shead.epos_controller.motion_in_progress = !epos_node->isTargetReached();
					} else {
						// Copy values to buffer.
						reply.shead.epos_controller.current = 0;
						reply.shead.epos_controller.motion_in_progress = false;
					}

					// Calculate current joint values.
					get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

					// Copy values to buffer.
					reply.shead.epos_controller.position = current_joints[0];
					break;
				default:
					// Throw non-fatal error - command not supported.
					BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_command());
					break;
			}
		}

		reply.servo_step = step_counter;

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

void effector::create_kinematic_models_for_given_robot(void)
{
	add_kinematic_model(new kinematics::shead::model());
	set_kinematic_model(0);
}

/*--------------------------------------------------------------------------*/
void effector::create_threads()
{
	rb_obj = (boost::shared_ptr <common::reader_buffer>) new common::reader_buffer(*this);
	//vis_obj = (boost::shared_ptr <common::vis_server>) new common::vis_server(*this);
}

lib::INSTRUCTION_TYPE effector::receive_instruction()
{
	return common::effector::receive_instruction(instruction);
}

void effector::variant_reply_to_instruction()
{
	reply_to_instruction(reply);
}

} // namespace shead
} // namespace edp
} // namespace mrrocpp

