#include <cstdio>
#include <iostream>
#include <bitset>
//#include <boost/range/algorithm.hpp>
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/foreach.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

#include "edp_e_smb.h"
#include "festo_and_inputs.h"
#include "base/edp/reader.h"
#include "robot/smb/kinematic_model_smb.h"
#include "base/edp/manip_trans_t.h"
#include "base/edp/vis_server.h"

#include "exceptions.h"

using namespace std;

namespace mrrocpp {
namespace edp {
namespace smb {

#include "base/lib/debug.hpp"

// Access to kinematic parameters.
#define PARAMS ((mrrocpp::kinematics::smb::model*)this->get_current_kinematic_model())

// Maximum velocity: legs (verified for 2000 rpm), pkm (verified for 2000 rpm).
const uint32_t effector::Vdefault[lib::smb::NUM_OF_SERVOS] = { 300UL, 1000UL };
// Maximum acceleration: legs (verified for 4000 rpm/s), pkm (verified for 4000 rpm/s).
const uint32_t effector::Adefault[lib::smb::NUM_OF_SERVOS] = { 300UL, 1000UL };
// Maximum deceleration: legs (verified for 4000 rpm/s), pkm (verified for 4000 rpm/s).
const uint32_t effector::Ddefault[lib::smb::NUM_OF_SERVOS] = { 300UL, 1000UL };

effector::effector(common::shell &_shell, lib::robot_name_t l_robot_name) :
		motor_driven_effector(_shell, l_robot_name, instruction, reply), cleaning_active(false)
{
	DEBUG_METHOD;

	number_of_servos = lib::smb::NUM_OF_SERVOS;

	cleaning_active = config.exists_and_true("cleaning_active");

	// Check whether rotation of the PKM (upper SMB platform) is disabled or enabled.
	pkm_rotation_disabled = config.exists_and_true("pkm_rotation_disabled");

	// Create manipulator kinematic model.
	create_kinematic_models_for_given_robot();

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
		legs_rotation_node = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 8, "Legs");
		pkm_rotation_node = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 9, "PKM");

		// Collect axes into common array container.
		axes[0] = &(*legs_rotation_node);
		axes[1] = &(*pkm_rotation_node);

		// Create festo node.
		cpv10 = (boost::shared_ptr <festo::cpv>) new festo::cpv(*gateway, FESTO_ADRESS);

	}

}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	DEBUG_METHOD;

	motor_driven_effector::single_thread_master_order(nm_task, nm_tryb);
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

		// If PKM motor enabled, perform the synchronization.
		if (!pkm_rotation_disabled) {
			// Step 1: Setup velocity control with analog setpoint.
			pkm_rotation_node->setOperationMode(maxon::epos::OMD_VELOCITY_MODE);

			// Velocity and acceleration limits.
			pkm_rotation_node->setMaxProfileVelocity(50);
			pkm_rotation_node->setMaxAcceleration(1000);

			// NOTE: We assume, that scaling and offset are already set in the EPOS2.

			// Start motion.
			pkm_rotation_node->setControlword(0x000F);

			// Enable analog velocity setpoint.
			pkm_rotation_node->setAnalogInputFunctionalitiesExecutionMask(false, true, false);

			// Setup timer for monitoring.
			boost::system_time wakeup = boost::get_system_time();

			// Loop until reaching zero offset.
			printf("\n");
			while (pkm_rotation_node->getAnalogInput1() != pkm_zero_position_voltage) {
				printf("\rSMB calibration: Desired =%+10d[mv] | Actual  =%+10d[mv] | Offset  =%+10d[mv]", (int) pkm_zero_position_voltage, (int) pkm_rotation_node->getAnalogInput1(), (int) pkm_rotation_node->getAnalogVelocitySetpoint());
				fflush(stdout);

				// Sleep for a constant period of time
				wakeup += boost::posix_time::milliseconds(5);

				boost::thread::sleep(wakeup);
			}
			printf("\n");

			// Disable analog velocity setpoint.
			pkm_rotation_node->setAnalogInputFunctionalitiesExecutionMask(false, false, false);

			// Restore velocity and acceleration limits.
			pkm_rotation_node->setMaxProfileVelocity(Vdefault[1]);
			pkm_rotation_node->setMaxAcceleration(Adefault[1]);

			// Step 2: Homing.
			// Activate homing mode.
			pkm_rotation_node->doHoming(maxon::epos::HM_INDEX_POSITIVE_SPEED, pkm_zero_position_offset);

			// Homing of the motor controlling the legs rotation - set current position as 0.
			legs_relative_zero_position = legs_rotation_node->getActualPosition();

			// Set *extended* limits for PKM rotation.
			axes[1]->setMinimalPositionLimit(PARAMS->lower_pkm_motor_pos_limits - 1000);
			axes[1]->setMaximalPositionLimit(PARAMS->upper_pkm_motor_pos_limits + 1000);
		} //: if pkm enabled.

		// Check whether the synchronization was successful.
		check_controller_state();

		// Throw non-fatal error - if synchronization wasn't successful.
		if (!controller_state_edp_buf.is_synchronised) {
			BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::fe_synchronization_unsuccessful());
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

void effector::check_controller_state()
{
	DEBUG_METHOD;

	if (robot_test_mode) {
		// In test mode robot is always synchronized.
		controller_state_edp_buf.is_synchronised = true;
		return;
	}

	// Try to get state of each axis.
	unsigned int powerOn = 0;
	unsigned int enabled = 0;

	// Check axes.
	for (size_t i = 0; i < axes.size(); ++i) {
		try {
			cout << string("Axis ") << axes[i]->getDeviceName() << endl;
			// Get current EPOS state.
			maxon::epos::actual_state_t state = axes[i]->getState();
			if (state != maxon::epos::OPERATION_ENABLE) {
				// Print state.
				axes[i]->printState();
				// Check if in the FAULT state.
				if (state == maxon::epos::FAULT) {
					// Read number of errors.
					int errNum = axes[i]->getNumberOfErrors();

					// Iterate over error array.
					for (size_t j = 1; j <= errNum; ++j) {
						// Get the detailed error code.
						uint32_t errCode = axes[i]->getErrorHistory(j);
						// Send message to SR.
						msg->message(mrrocpp::lib::FATAL_ERROR, string("Axis ") + axes[i]->getDeviceName() + ": "
								+ axes[i]->ErrorCodeMessage(errCode));
					}
				} else if (state == maxon::epos::SWITCH_ON_DISABLED) {
					// Display this information only in the case of legs rotation or (in case of pkm rotation) if it is enabled.
					if ((i == 0) || (!pkm_rotation_disabled)) {
						// Send message to SR.
						msg->message(mrrocpp::lib::FATAL_ERROR, string("Epos controlling ") + axes[i]->getDeviceName()
								+ " rotation is disabled");
					}
				} //: if fault || disabled
			} else {
				// EPOS in enabled state.
				enabled++;
			}
			// At this point we are sure that axis is working, this powered.
			powerOn++;
		} catch (...) {
			// Probably the axis is not powered on, do nothing.
		}
	} //: for

	if (pkm_rotation_disabled) {
		// If motor is disabled - robot is always synchronised.
		controller_state_edp_buf.is_synchronised = true;
		// ... and at least only the legs rotation must be enabled.
		controller_state_edp_buf.robot_in_fault_state = (enabled < 1);
	} else {
		// Robot is synchronized if only one axis - the one controlling the PKM rotation - is referenced.
		controller_state_edp_buf.is_synchronised = pkm_rotation_node->isReferenced();
		// Check fault state.
		controller_state_edp_buf.robot_in_fault_state = (enabled != axes.size());
	}
	// Check whether robot is powered.
	controller_state_edp_buf.is_power_on = (powerOn == axes.size());
}

void effector::get_controller_state(lib::c_buffer &instruction_)
{
	DEBUG_METHOD;

	try {
		// False is the initial value.
		controller_state_edp_buf.is_synchronised = false;
		controller_state_edp_buf.is_power_on = false;
		controller_state_edp_buf.robot_in_fault_state = false;

		// Check controller state.
		check_controller_state();

		// Disable PKM rotation if required.
		if (!robot_test_mode) {
			if (pkm_rotation_disabled) {
				if (pkm_rotation_node->getState() != maxon::epos::SWITCH_ON_DISABLED) {
					// Disable the motor (in fact move to the SWITCH_ON_DISABLED state).
					pkm_rotation_node->setState(maxon::epos::DISABLE_VOLTAGE);
				}
				// Send adequate message to the UI.
				msg->message(string("Epos controlling ") + pkm_rotation_node->getDeviceName()
						+ string(" rotation is disabled"));
			}
		}

		// Copy data to the reply buffer.
		reply.controller_state = controller_state_edp_buf;

		// Initiate motor positions.
		for (size_t i = 0; i < axes.size(); ++i) {
			// If this is not a test mode and robot is synchronized.
			if (!robot_test_mode && is_synchronised())
				// Get actual motor positions.
				current_motor_pos[i] = axes[i]->getActualPosition();
			else
				// Zero all motor positions.
				current_motor_pos[i] = 0;
		}
#if(DEBUG_MOTORS)
		cout << "current_motor_pos: " << current_motor_pos.transpose() << "\n";
#endif

		// Compute current motor positions on the base of current motors.
		get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

#if(DEBUG_JOINTS)
		cout << "current_joints: " << current_joints.transpose() << "\n";
#endif

		// Reset zero position.
		if ((!robot_test_mode)
				&& ((current_legs_state() == lib::smb::ALL_IN) || (current_legs_state() == lib::smb::ALL_OUT))) {
			// Set current position as 0.
			legs_relative_zero_position = (int32_t) current_motor_pos[0];
		} else
			legs_relative_zero_position = 0;

#if(DEBUG_MOTORS)
		cout << "legs_relative_zero_position: " << legs_relative_zero_position << "\n";
#endif

		// Lock data structure during update.
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

lib::smb::ALL_LEGS_VARIANT effector::current_legs_state(void)
{
	return fai->current_legs_state;
}

lib::smb::ALL_LEGS_VARIANT effector::next_legs_state(void)
{
	return fai->next_legs_state;
}

void effector::move_arm(const lib::c_buffer &instruction_)
{
	DEBUG_METHOD;

	try {
		switch (instruction.smb.variant)
		{
			case lib::smb::POSE:
				DEBUG_COMMAND("POSE");
				// Control the two SMB rotational motors.
				// Parse command.
				parse_motor_command();
				// Execute motion.
				execute_motor_motion();
				break;

			case lib::smb::QUICKSTOP:
				DEBUG_COMMAND("QUICKSTOP");

				if (!robot_test_mode) {
					// Execute command
					BOOST_FOREACH(maxon::epos * node, axes)
							{
								// Brake with Quickstop command
								node->setState(maxon::epos::QUICKSTOP);
							}

					// Reset nodes right after.
					legs_rotation_node->reset();
					// Reset pkm only if enabled.
					if (!pkm_rotation_disabled)
						pkm_rotation_node->reset();
				} //: !test_mode
				break;

			case lib::smb::CLEAR_FAULT:
				DEBUG_COMMAND("CLEAR_FAULT");

				if (!robot_test_mode) {
					BOOST_FOREACH(maxon::epos * node, axes)
							{
								node->clearFault();
							}
					// If PKM rotation motor should be disabled.
					if (pkm_rotation_disabled)
						// Disable the motor (in fact move to the SWITCH_ON_DISABLED state).
						pkm_rotation_node->setState(maxon::epos::DISABLE_VOLTAGE);
				} //: !test_mode
				break;

			case lib::smb::FESTO:
				DEBUG_COMMAND("FESTO");

				// Check state of the robot.
				if (controller_state_edp_buf.robot_in_fault_state)
					BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::fe_robot_in_fault_state());

				//if (is_base_positioned_to_move_legs)
				fai->command();
				// If all legs are currently OUT then reset legs rotation.
				if (current_legs_state() == lib::smb::ALL_OUT) {
					// Set current position as zero.
					if (!robot_test_mode) {
						legs_relative_zero_position = legs_rotation_node->getActualPosition();
					}
				}
				break;

			default:
				// Throw non-fatal error - invalid command.
				BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_command());
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

void effector::parse_motor_command()
{
	DEBUG_METHOD;

	// Check state of the robot.
	if (controller_state_edp_buf.robot_in_fault_state)
		BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::fe_robot_in_fault_state());

	// The TWO_UP_ONE_DOWN is the only state in which control of both motors (legs and SPKM rotations) is possible.
	// In other states control of the motor rotating the legs (lower SMB motor) is prohibited!
	if (current_legs_state() != lib::smb::TWO_IN_ONE_OUT) {
		// Check the difference between current and desired values.
		// Check motors.
		if ((instruction.smb.set_pose_specification == lib::smb::MOTOR)
				&& (current_motor_pos[0] != instruction.smb.motor_pos[0]))
			BOOST_THROW_EXCEPTION(mrrocpp::edp::smb::nfe_legs_rotation_prohibited_in_given_state()<<current_state(current_legs_state()));
		// Check joints.
		// TODO: !check eps instead of classic comparison!
		else if ((instruction.smb.set_pose_specification == lib::smb::JOINT)
				&& (current_joints[0] != instruction.smb.joint_pos[0]))
			BOOST_THROW_EXCEPTION(mrrocpp::edp::smb::nfe_legs_rotation_prohibited_in_given_state()<<current_state(current_legs_state()));
		// Check externals.
		else if ((instruction.smb.set_pose_specification == lib::smb::EXTERNAL)
				&& (current_joints[0]
						!= instruction.smb.base_vs_bench_rotation * mrrocpp::kinematics::smb::leg_rotational_ext2i_ratio))
			BOOST_THROW_EXCEPTION(mrrocpp::edp::smb::nfe_legs_rotation_prohibited_in_given_state()<<current_state(current_legs_state()));
	}

	// Interpret command according to the pose specification.
	switch (instruction.smb.set_pose_specification)
	{
		case lib::smb::MOTOR:
			DEBUG_COMMAND("MOTOR");

			// Copy data directly from buffer.
			for (int i = 0; i < number_of_servos; ++i) {
				desired_motor_pos_new[i] = instruction.smb.motor_pos[i];
			}
#if(DEBUG_MOTORS)
			cout << "MOTOR: " << desired_motor_pos_new.transpose() << endl;
#endif
			// Check the desired motor (only motors!) values if they are absolute.
			get_current_kinematic_model()->check_motor_position(desired_motor_pos_new);
			// Transform desired motors to joints.
			get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, desired_joints);
			break;
		case lib::smb::JOINT:
			DEBUG_COMMAND("JOINT");

			// Copy data directly from buffer.
			for (int i = 0; i < number_of_servos; ++i) {
				desired_joints[i] = instruction.smb.joint_pos[i];
			}
#if(DEBUG_JOINTS)
			cout << "JOINT: " << desired_joints.transpose() << endl;
#endif

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
		case lib::smb::EXTERNAL:
			DEBUG_COMMAND("EXTERNAL");
			// Leg rotational joint: Copy data directly from buffer and recalculate joint value.
			desired_joints[0] = instruction.smb.base_vs_bench_rotation
					* mrrocpp::kinematics::smb::leg_rotational_ext2i_ratio;
			// SPKM rotational joint: Copy data joint value directly from buffer.
			desired_joints[1] = instruction.smb.pkm_vs_base_rotation;
#if(DEBUG_JOINTS)
			cout << "JOINT[0]: " << desired_joints[0] << endl;
			cout << "JOINT[1]: " << desired_joints[1] << endl;
#endif

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
	} //: switch (instruction.smb.set_pose_specification)
}

void effector::execute_motor_motion()
{
	DEBUG_METHOD;

	// Perform motion depending on its type.
	// Note: at this point we assume, that desired_motor_pos_new holds a validated data.

	// Execute command.
	if (is_synchronised()) {
#if(DEBUG_MOTORS)
		cout << "MOTOR: absolute:" << desired_motor_pos_new.transpose() << endl;
#endif
		// Robot is synchronized.
		if (!robot_test_mode) {
			// Perform legs rotation.
			// Set velocity and acceleration values.
			legs_rotation_node->setProfileVelocity(Vdefault[0]);
			legs_rotation_node->setProfileAcceleration(Adefault[0]);
			legs_rotation_node->setProfileDeceleration(Ddefault[0]);
			// In case of legs rotation node perform the *absolute-relative* move.
			legs_rotation_node->moveAbsolute(desired_motor_pos_new[0] + legs_relative_zero_position);

			// Perform pkm rotation only when motor is enabled.
			if (!pkm_rotation_disabled) {
				// Set velocity and acceleration values.
				pkm_rotation_node->setProfileVelocity(Vdefault[1]);
				pkm_rotation_node->setProfileAcceleration(Adefault[1]);
				pkm_rotation_node->setProfileDeceleration(Ddefault[1]);
				pkm_rotation_node->moveAbsolute(desired_motor_pos_new[1]);
			}
		} else {
			for (size_t i = 0; i < axes.size(); ++i) {
				// Virtually "move" to desired absolute position.
				current_motor_pos[i] = desired_motor_pos_new[i];
			} //: for
		}
	} else {
#if(DEBUG_MOTORS)
		cout << "MOTOR: relative:" << desired_motor_pos_new.transpose() << endl;
#endif
		// Robot unsynchronized.
		if (!robot_test_mode) {
			// Perform legs rotation.
			// Set velocity and acceleration values.
			legs_rotation_node->setProfileVelocity(Vdefault[0]);
			legs_rotation_node->setProfileAcceleration(Adefault[0]);
			legs_rotation_node->setProfileDeceleration(Ddefault[0]);
			// In case of legs rotation node perform the relative move.
			legs_rotation_node->moveRelative(desired_motor_pos_new[0]);

			// Perform pkm rotation only when motor is enabled.
			if (!pkm_rotation_disabled) {
				// Set velocity and acceleration values.
				pkm_rotation_node->setProfileVelocity(Vdefault[1]);
				pkm_rotation_node->setProfileAcceleration(Adefault[1]);
				pkm_rotation_node->setProfileDeceleration(Ddefault[1]);
				pkm_rotation_node->moveRelative(desired_motor_pos_new[1]);
			}
		} else {
			for (size_t i = 0; i < axes.size(); ++i) {
				// Virtually "move" to desired relative position.
				current_motor_pos[i] += desired_motor_pos_new[i];
			}
		} //: for
	} //: is_synchronised

}

void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction_)
{
	DEBUG_METHOD;

	try {
		// Check controller state.
		check_controller_state();

		// Handle only GET and SETGET instructions.
		if (instruction.instruction_type != lib::SET) {
			switch (instruction.smb.get_pose_specification)
			{

				case lib::smb::MOTOR:
					DEBUG_COMMAND("MOTOR");

					// For every axis.
					for (size_t i = 0; i < number_of_servos; ++i) {
						if (!robot_test_mode) {
							// Update current position.
							current_motor_pos[i] = axes[i]->getActualPosition();
							// In case of legs rotation node...
							if (i == 0)
								// ... compute relative position.
								current_motor_pos[i] -= legs_relative_zero_position;
							// Copy values to buffer.
							reply.smb.epos_controller[i].position = current_motor_pos[i];
							reply.smb.epos_controller[i].current = axes[i]->getActualCurrent();
							reply.smb.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
						} else {
							// Copy values to buffer.
							reply.smb.epos_controller[i].position = current_motor_pos[i];
							reply.smb.epos_controller[i].current = 0;
							reply.smb.epos_controller[i].motion_in_progress = false;
						}
					}
					break;
				case lib::smb::JOINT:

					// For every axis.
					for (size_t i = 0; i < axes.size(); ++i) {
						if (!robot_test_mode) {
							// Update current position.
							current_motor_pos[i] = axes[i]->getActualPosition();
							// In case of legs rotation node...
							if (i == 0)
								// ... compute relative position.
								current_motor_pos[i] -= legs_relative_zero_position;
							// Copy values to buffer.
							reply.smb.epos_controller[i].current = axes[i]->getActualCurrent();
							reply.smb.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
						} else {
							// Copy values to buffer.
							reply.smb.epos_controller[i].current = 0;
							reply.smb.epos_controller[i].motion_in_progress = false;
						}
					}

					// Calculate current joint values.
					get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

					// Copy values to buffer.
					for (int i = 0; i < number_of_servos; ++i) {
						reply.smb.epos_controller[i].position = current_joints[i];
					}
					break;
				case lib::smb::EXTERNAL:
					DEBUG_COMMAND("EXTERNAL");

					// For every axis.
					for (size_t i = 0; i < axes.size(); ++i) {
						if (!robot_test_mode) {
							// Update current position.
							current_motor_pos[i] = axes[i]->getActualPosition();
							// In case of legs rotation node...
							if (i == 0)
								// ... compute relative position.
								current_motor_pos[i] -= legs_relative_zero_position;
							// Copy values to buffer.
							reply.smb.epos_controller[i].current = axes[i]->getActualCurrent();
							reply.smb.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
						} else {
							// Copy values to buffer.
							reply.smb.epos_controller[i].current = 0;
							reply.smb.epos_controller[i].motion_in_progress = false;
						}
					}

					// Calculate current joint values.
					get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

					// Leg rotational joint: recalculate joint value and copy data to buffer.
					reply.smb.epos_controller[0].position = current_joints[0]
							/ mrrocpp::kinematics::smb::leg_rotational_ext2i_ratio;
					// SPKM rotational joint: copy data to buffer.
					reply.smb.epos_controller[1].position = current_joints[1];
					break;
				default:
					// Throw non-fatal error - command not supported.
					BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_command());
					break;
			}
		}

		// SMB clamps.
		fai->create_reply();
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
	// Create basic kinematic mode.
	add_kinematic_model(new kinematics::smb::model());
	// Set is as active.
	set_kinematic_model(0);
}

void effector::create_threads()
{
	fai = new festo_and_inputs(*this);
	rb_obj = (boost::shared_ptr <common::reader_buffer>) new common::reader_buffer(*this);
	// vis_obj = (boost::shared_ptr <common::vis_server>) new common::vis_server(*this);
}

lib::INSTRUCTION_TYPE effector::receive_instruction()
{
	return common::effector::receive_instruction(instruction);
}

void effector::variant_reply_to_instruction()
{
	reply_to_instruction(reply);
}

} // namespace smb
} // namespace edp
} // namespace mrrocpp

