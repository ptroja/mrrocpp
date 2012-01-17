#include <iostream>
#include <fstream>
#include <cstdio>
#include <sys/stat.h>
//<sys/types.h>
#include <boost/foreach.hpp>
#include <boost/static_assert.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

#include "edp_e_spkm.h"
#include "base/edp/reader.h"

#include "kinematic_model_spkm.h"
#include "base/edp/manip_trans_t.h"

#include "robot/canopen/gateway_epos_usb.h"
#include "robot/canopen/gateway_socketcan.h"
#include "robot/maxon/epos.h"

#include "base/lib/pvt.hpp"
#include "base/lib/pvat_cartesian.hpp"

#include "exceptions.h"
#include "robot/maxon/epos_exceptions.hpp"

#include "robot/maxon/ipm_executor.h"

namespace mrrocpp {
namespace edp {
namespace spkm {

#include "base/lib/debug.hpp"
// Debug PVT triples.
#define DEBUG_PVT 1

using namespace mrrocpp::lib;
using namespace mrrocpp::lib::pvat;
using namespace std;

// Access to kinematic parameters.
#define PARAMS ((mrrocpp::kinematics::spkm::kinematic_model_spkm*)this->get_current_kinematic_model())->get_kinematic_parameters()

// Initialize the limit extension.
const uint32_t effector::limit_extension = 1000;


effector::effector(common::shell &_shell, lib::robot_name_t l_robot_name) :
		manip_effector(_shell, l_robot_name, instruction, reply)
{
	DEBUG_METHOD;

	// Set number of servos.
	number_of_servos = lib::spkm::NUM_OF_SERVOS;

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

		// Epos objects initialization moved to the SPKM1 and SPKM2 constructors.
	}
}

void effector::disable_moog_motor()
{
	// Disable operation of the Moog motor if it is stopped to activate brake.
	if(!axis2->isTargetReached()) {
		msg->message("Disabling the Moog motor not allowed during motion.");
		BOOST_THROW_EXCEPTION(exception::fe());
	}

	msg->message("Disabling Moog motor");
	axis2->setState(maxon::epos::DISABLE_OPERATION);

	// Setup the wakeup time
	boost::system_time wakeup = boost::get_system_time();
	const boost::system_time timeout = wakeup + boost::posix_time::milliseconds(1000);

	// Condition to monitor for
	bool in_switched_on = false;

	// Monitor until state change.
	while(!in_switched_on) {
		// Increment the wakeup time.
		wakeup += boost::posix_time::milliseconds(10);

		// Check time clock.
		if(wakeup > timeout) {
			msg->message("Timeout waiting to brake the moog motor.");
			BOOST_THROW_EXCEPTION(exception::fe());
		}

		// Wait for device state to change
		boost::thread::sleep(wakeup);

		maxon::epos::actual_state_t state = axis2->getState();

		switch (state) {
			// These are expected transition states
			case maxon::epos::OPERATION_ENABLE:
				// Still disabling, do nothing.
				break;
			case maxon::epos::SWITCHED_ON:
				in_switched_on = true;
				break;
			case maxon::epos::FAULT:
				BOOST_THROW_EXCEPTION(exception::fe_robot_in_fault_state());
				break;
			default:
				std::cout << "Moog node transited to state '" << maxon::epos::stateDescription(state)
									<< "' during braking" << std::endl;
				break;
		}
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
	unsigned int referenced = 0;
	unsigned int powerOn = 0;
	unsigned int enabled = 0;

	boost::array <canopen::WORD, lib::spkm::NUM_OF_SERVOS> cachedStatusWords;

	// Check axes.
	for (size_t i = 0; i < axes.size(); ++i) {
		try {
			// Get current status.
			cachedStatusWords[i] = axes[i]->getStatusWord();
			// Get current epos state.
			maxon::epos::actual_state_t state = maxon::epos::status2state(cachedStatusWords[i]);
			if (axes[i] == axis2) {
				switch (state)
				{
					//case maxon::epos::SWITCH_ON_DISABLED:
					case maxon::epos::SWITCHED_ON:
					case maxon::epos::REFRESH:
					case maxon::epos::MEASURE_INIT:
					case maxon::epos::OPERATION_ENABLE:
						// We are happy with these states.
						enabled++;
						break;
					case maxon::epos::FAULT:
						// Print state.
						axes[i]->printState();
						{
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
						}
						break;
					default:
						// Print state.
						axes[i]->printState();
						msg->message(mrrocpp::lib::FATAL_ERROR, string("Epos controlling ") + axes[i]->getDeviceName()
								+ " is not disabled & braked");
						break;
				}
			} else {
				if (state != maxon::epos::OPERATION_ENABLE) {
					// Print state.
					axes[i]->printState();
					// Check if in the FAULT state
					if (state == maxon::epos::FAULT) {
						// Read number of errors
						int errNum = axes[i]->getNumberOfErrors();
						for (size_t j = 1; j <= errNum; ++j) {
							// Get the detailed error
							uint32_t errCode = axes[i]->getErrorHistory(j);
							// Send message to SR.
							msg->message(mrrocpp::lib::FATAL_ERROR, string("Axis ") + axes[i]->getDeviceName() + ": "
									+ axes[i]->ErrorCodeMessage(errCode));
						}
					} else if (state == maxon::epos::SWITCH_ON_DISABLED) {
						// Send message to SR.
						msg->message(mrrocpp::lib::FATAL_ERROR, string("Epos controlling ") + axes[i]->getDeviceName()
								+ " is disabled");
					} else {
						// Send message to SR.
						msg->message(mrrocpp::lib::FATAL_ERROR, string("Epos controlling ") + axes[i]->getDeviceName()
								+ " is not enabled");
					}
				} else {
					// EPOS in enabled state.
					enabled++;
				}
			}
			if (maxon::epos::isReferenced(cachedStatusWords[i])) {
				// Do not break from this loop so this is a also a preliminary axis error check
				referenced++;
			}
			powerOn++;
		} catch (...) {
			// Probably the axis is not powered on, do nothing.
		}
	}
	// Robot is synchronized if all axes are referenced.
	controller_state_edp_buf.is_synchronised = (referenced == axes.size());
	// Check whether all axes are powered on.
	controller_state_edp_buf.is_power_on = (powerOn == axes.size());
	// Check fault state.
	controller_state_edp_buf.robot_in_fault_state = (enabled != axes.size());

	// If error was detected in any motor - terminate motion.
	if (controller_state_edp_buf.robot_in_fault_state) {
		// Stop only motors which are moving at the moment.
		for (size_t i = 0; i < axes.size(); ++i) {
			if (maxon::epos::isTargetReached(cachedStatusWords[i])) {
				// Stop the motion.
				axes[i]->setState(maxon::epos::DISABLE_VOLTAGE);
			}
		}
		BOOST_THROW_EXCEPTION(exception::fe_robot_in_fault_state());
	}
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

		// FIXME: uncomment the following line to allow multiple synchronization without resetting.
		//controller_state_edp_buf.is_synchronised = false;

		// Copy data to reply buffer
		reply.controller_state = controller_state_edp_buf;

		// Initiate motor positions.
		for (size_t i = 0; i < axes.size(); ++i) {
			// If this is a test mode or robot isn't synchronized.
			if (robot_test_mode || !is_synchronised()) {
				// Zero all motor positions.
				current_motor_pos[i] = 0;

				// Reset limits.
				if(!robot_test_mode) {
					BOOST_FOREACH(boost::shared_ptr<maxon::epos> node, axes) {
						// Disable both limits.
						node->disablePositionLimits();
					}
				}
			} else {
				// Get actual motor positions.
				current_motor_pos[i] = axes[i]->getActualPosition();
			}
		}
#if(DEBUG_MOTORS)
		std::cerr << "current_motor_pos: " << current_motor_pos.transpose() << "\n";
#endif

		// Compute current motor positions on the base of current motors.
		get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
#if(DEBUG_JOINTS)
		std::cerr << "current_joints: " << current_joints.transpose() << "\n";
#endif

		// Reset cartesian-related variables.
		current_end_effector_frame.setIdentity();
		desired_end_effector_frame.setIdentity();
		is_current_cartesian_pose_known = false;

		// Try to read tool (SHEAD) transformation from the configuration file.
		try {
			shead_frame.set(config.value <std::string>("shead_frame"));
		} catch (std::exception& e_) {
			// Print failure reason.
			std::cerr << e_.what() << endl;
			// Set identity.
			shead_frame.setIdentity();
		}

#if(DEBUG_FRAMES)
		std::cerr.precision(8);
		std::cerr << "shead_frame: " << shead_frame << endl;
		std::cerr.precision(8);
		std::cerr << "shead_frame * !shead_frame: " << shead_frame * !shead_frame << endl;
#endif

		// Lock data structure during update.
		{
			boost::mutex::scoped_lock lock(effector_mutex);

			// Initialize internal data.
			servo_current_motor_pos = current_motor_pos;
			desired_motor_pos_old = current_motor_pos;
			desired_motor_pos_new = current_motor_pos;
			desired_joints = current_joints;
			desired_joints_old = current_joints;
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

void effector::synchronise(void)
{
	DEBUG_METHOD;

	try {
		// WORKAROUND: remove those two lines!
//		synchronise_moog_motor(*axis2, PARAMS.lower_motor_pos_limits[2], PARAMS.upper_motor_pos_limits[2], PARAMS.moog_motor_homing_offset);
//		return;

		if (robot_test_mode) {
			controller_state_edp_buf.is_synchronised = true;
			return;
		}

		// Check state of the robot.
		if (controller_state_edp_buf.robot_in_fault_state)
			BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::fe_robot_in_fault_state());

		// Switch linear axes to homing mode.
		axisA->setOperationMode(maxon::epos::OMD_HOMING_MODE);
		axisB->setOperationMode(maxon::epos::OMD_HOMING_MODE);
		axisC->setOperationMode(maxon::epos::OMD_HOMING_MODE);

		// Enable controller.
		axisA->reset();
		axisB->reset();
		axisC->reset();

		// Start homing.
		axisA->startHoming();
		axisB->startHoming();
		axisC->startHoming();

		// Loop until homing is finished
		bool finished;
		do {
			finished = true;
			if (!axisA->isHomingFinished()) finished = false;
			if (!axisB->isHomingFinished()) finished = false;
			if (!axisC->isHomingFinished()) finished = false;
			// Delay between queries
			usleep(20000);
		} while (!finished);

		// Do homing for Moog motor.
		synchronise_moog_motor(*axis2, PARAMS.lower_motor_pos_limits[2], PARAMS.upper_motor_pos_limits[2], PARAMS.moog_motor_homing_offset);

		// Do homing for another motor.
		axis1->setOperationMode(maxon::epos::OMD_HOMING_MODE);
		axis1->reset();
		axis1->startHoming();

		// Wait until second homing is finished.
		while(!axis1->isHomingFinished()) {
			// Delay between queries.
			usleep(20000);
		}

		// Do homing for yet another motor.
		axis3->setOperationMode(maxon::epos::OMD_HOMING_MODE);
		axis3->reset();
		axis3->startHoming();

		// Wait until second homing is finished.
		while(!axis3->isHomingFinished()) {
			// Delay between queries.
			usleep(20000);
		}

		// Reset internal state of the motor positions
		for (size_t i = 0; i < number_of_servos; ++i) {
			current_motor_pos[i] = desired_motor_pos_old[i] = 0;
		}

		// Set *extended* limits.
		for (size_t i = 0; i < axes.size(); ++i) {
			axes[i]->setMinimalPositionLimit(PARAMS.lower_motor_pos_limits[i] - limit_extension);
			axes[i]->setMaximalPositionLimit(PARAMS.upper_motor_pos_limits[i] + limit_extension);
		}

		// Compute joints positions in the home position
		get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

		// Now the robot is synchronised.
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


void effector::synchronise_moog_motor(maxon::epos & epos_, int32_t  negative_limit_, int32_t  positive_limit_, int32_t homing_offset)
{
	try{
		// Disable both limits.
		epos_.disablePositionLimits();

		// Velocity mode in the direction of negative limit.
		epos_.setOperationMode(maxon::epos::OMD_VELOCITY_MODE);
		epos_.reset();

		// TODO: set max acceleration?
		epos_.setControlword(0x010f);
		epos_.setVelocityModeSettingValue(-100);

		// Start monitoring after some interval for acceleration.
		boost::system_time wakeup = boost::get_system_time() + boost::posix_time::milliseconds(25);

		// Startup monitoring counter.
		unsigned int monitor_counter = 0;

		do {
			// Wait for device state to change.
			boost::thread::sleep(wakeup);

			// Increment the next wakeup time.
			wakeup += boost::posix_time::milliseconds(5);

			if(++monitor_counter < 20) {
				// FIXME: Uncomment the following to debug the wakup/startup timer.
				// std::cout << "Moog motor velocity: " << (int) epos_.getActualVelocityAveraged() << std::endl;
			}
		} while(epos_.getActualVelocityAveraged() < -10);

		// Halt.
		epos_.reset();

		try {
			// Homing: move to the index, then continue with an offset.
			epos_.doHoming(maxon::epos::HM_INDEX_POSITIVE_SPEED, homing_offset);
			epos_.monitorHomingStatus();
		} catch (boost::exception &e_) {
			// Motor jam!
			BOOST_THROW_EXCEPTION(fe_motor_jam_detected() << device_name(epos_.getDeviceName()));
		}


		// Revert to the original limits.
		epos_.setMinimalPositionLimit(negative_limit_ - limit_extension);
		epos_.setMaximalPositionLimit(positive_limit_ + limit_extension);
	} catch (...) {
		// Revert to the original limits anyway.
		epos_.setMinimalPositionLimit(negative_limit_ - limit_extension);
		epos_.setMaximalPositionLimit(positive_limit_ + limit_extension);
		// Rethrow the exception.
		throw;
	}
}



void effector::move_arm(const lib::c_buffer &instruction_)
{
	DEBUG_METHOD;

	try {
		// Check command type.
		switch (instruction.spkm.variant)
		{
			case lib::spkm::POSE:
				DEBUG_COMMAND("POSE");
				if (controller_state_edp_buf.robot_in_fault_state) {
					return;
				}

				// Special case: operational motion.
				if (instruction.spkm.motion_variant == lib::epos::OPERATIONAL) {
					DEBUG_COMMAND("OPERATIONAL");

					interpolated_motion_in_operational_space();
					// Continue - update the robot state.
					break;
				}
				// Parse command.
				parse_motor_command();
				// Execute motion.
				execute_motion();
				// Continue - update the robot state.
				break;
			case lib::spkm::QUICKSTOP:
				DEBUG_COMMAND("QUICKSTOP");

				if (!robot_test_mode) {
					// Execute command
					BOOST_FOREACH(boost::shared_ptr<maxon::epos> node, axes)
							{
								// Brake with Quickstop command
								node->setState(maxon::epos::QUICKSTOP);
							}

					// Reset node right after.
					BOOST_FOREACH(boost::shared_ptr<maxon::epos> node, axes)
							{
								// Brake with Quickstop command
								node->reset();
							}
				}
				// Internal position counters need not be updated.
				return;
			case lib::spkm::BRAKE:
				DEBUG_COMMAND("BRAKE");

				// Execute brake command.
				if(!robot_test_mode) {
					disable_moog_motor();
				}

				// Internal position counters need not be updated.
				return;
			case lib::spkm::CLEAR_FAULT:
				DEBUG_COMMAND("CLEAR_FAULT");

				BOOST_FOREACH(boost::shared_ptr<maxon::epos> node, axes)
						{
							node->clearFault();
						}
				// Internal position counters need not be updated.
				return;
			default:
				// Throw non-fatal error - invalid command.
				BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_command());
				break;
		}

		// Hold the issued command.
		desired_motor_pos_old = desired_motor_pos_new;

		// Check whether the motion was performed in the cartesian space - then we know where manipulator will be when the next command arrives:).
		if (instruction.spkm.set_pose_specification == lib::spkm::WRIST_XYZ_EULER_ZYZ) {
			// Command was given in the wrist frame.
			current_end_effector_frame = desired_end_effector_frame;
			current_spkm_frame = current_end_effector_frame * shead_frame;
			is_current_cartesian_pose_known = true;
#if(DEBUG_FRAMES)
			std::cerr.precision(8);
			std::cerr << "current_spkm_frame:\n" << current_spkm_frame << endl;
			std::cerr << "current_end_effector_frame:\n" << current_end_effector_frame << endl;
#endif
		} else if (instruction.spkm.set_pose_specification == lib::spkm::TOOL_XYZ_EULER_ZYZ) {
			// Command was given in the tool (SHEAD) frame.
			current_spkm_frame = desired_spkm_frame;
			current_end_effector_frame = desired_spkm_frame * !shead_frame;
			is_current_cartesian_pose_known = true;
#if(DEBUG_FRAMES)
			std::cerr.precision(8);
			std::cerr << "current_spkm_frame:\n" << current_spkm_frame << endl;
			std::cerr << "current_end_effector_frame:\n" << current_end_effector_frame << endl;
#endif
		} else {
			is_current_cartesian_pose_known = false;
			// This isn't required, because the flag contains major information.
//			current_end_effector_frame.setIdentity();
//			current_spkm_frame.setIdentity();
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

	try {
		switch (instruction.spkm.set_pose_specification)
		{
			case lib::spkm::MOTOR:
				DEBUG_COMMAND("MOTOR");

				// Copy data directly from buffer
				for (size_t i = 0; i < number_of_servos; ++i) {
					desired_motor_pos_new[i] = instruction.spkm.motor_pos[i];
				}
#if(DEBUG_MOTORS)
				std::cerr.precision(15);
				std::cerr << "MOTORS " << desired_motor_pos_new.transpose() << endl;
#endif

				if (is_synchronised()) {
					// Check the desired motor (only motors!) values if they are absolute.
					get_current_kinematic_model()->check_motor_position(desired_motor_pos_new);
				}

				break;

			case lib::spkm::JOINT:
				DEBUG_COMMAND("JOINT");

				// Copy data directly from buffer
				for (size_t i = 0; i < number_of_servos; ++i) {
					desired_joints[i] = instruction.spkm.joint_pos[i];
				}
#if(DEBUG_JOINTS)
				std::cerr.precision(15);
				std::cerr << "JOINTS " << desired_joints.transpose() << endl;
#endif

				if (!is_synchronised()) {
					// Throw non-fatal error - this mode requires synchronization.
					BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_robot_unsynchronized());
				}

				// Precondition - check whether the desired joint position is valid.
				get_current_kinematic_model()->check_joints(desired_joints);
				// Transform desired joint to motors (and check motors/joints values).
				get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new, desired_joints);
				// Postcondition - check whether the desired motor position is valid.
				get_current_kinematic_model()->check_motor_position(desired_motor_pos_new);

				break;

			case lib::spkm::WRIST_XYZ_EULER_ZYZ:
				DEBUG_COMMAND("XYZ_EULER_ZYZ");

#if(DEBUG_FRAMES)
				std::cerr << "XYZ_EULER_ZYZ: [";
				for (unsigned int i = 0; i < 6; ++i) {
					std::cerr.precision(8);
					std::cerr << instruction.spkm.goal_pos[i] << ", ";
				}
				std::cerr << "]\n";
#endif

				if (!is_synchronised())
					// Throw non-fatal error - this mode requires synchronization.
					BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_robot_unsynchronized());

				// Retrieve the desired homogeneous matrix on the base of received six  variables - a Euler Z-Y-Z representation.
				desired_end_effector_frame.set_from_xyz_euler_zyz_without_limits(Xyz_Euler_Zyz_vector(instruction.spkm.goal_pos));
//				desired_end_effector_frame.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(instruction.spkm.goal_pos));

#if(DEBUG_FRAMES)
				std::cerr.precision(8);
				std::cerr << desired_end_effector_frame << endl;
#endif
				// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
				get_current_kinematic_model()->inverse_kinematics_transform(desired_joints, desired_joints_old, desired_end_effector_frame);

				// Postcondition I - check desired Cartesian position, basing on the upper platform pose.
				get_current_kinematic_model()->check_cartesian_pose(desired_end_effector_frame);
				//get_current_kinematic_model()->check_joints(desired_joints);

				// Transform joints to motors.
				get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new, desired_joints);
#if(DEBUG_JOINTS)
				std::cerr.precision(15);
				std::cerr << "JOINTS " << desired_joints.transpose() << endl;
#endif

				// Postcondition II  - check whether the desired motor position is valid.
				get_current_kinematic_model()->check_motor_position(desired_motor_pos_new);

				// Remember the currently desired joints as old.
				desired_joints_old = desired_joints;
				break;

			case lib::spkm::TOOL_XYZ_EULER_ZYZ: {
				DEBUG_COMMAND("TOOL_XYZ_EULER_ZYZ");
#if(DEBUG_FRAMES)
				std::cerr << "TOOL_XYZ_EULER_ZYZ: [";
				for (unsigned int i = 0; i < 6; ++i) {
					std::cerr.precision(8);
					std::cerr << instruction.spkm.goal_pos[i] << ", ";
				}
				std::cerr << "]\n";
#endif
				if (!is_synchronised())
					// Throw non-fatal error - this mode requires synchronization.
					BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_robot_unsynchronized());

				// Retrieve the desired homogeneous matrix on the base of received six  variables - a Euler Z-Y-Z representation.
				desired_spkm_frame.set_from_xyz_euler_zyz_without_limits(Xyz_Euler_Zyz_vector(instruction.spkm.goal_pos));
//				desired_spkm_frame.set_from_xyz_rpy(lib::Xyz_Rpy_vector(instruction.spkm.goal_pos));
#if(DEBUG_FRAMES)
				std::cerr.precision(8);
				std::cerr << "SHEAD frame: " << desired_spkm_frame << endl;
#endif

				// Transform to the wrist frame.
				desired_end_effector_frame = desired_spkm_frame * !shead_frame;
#if(DEBUG_FRAMES)
				std::cerr.precision(8);
				std::cerr << "Wrist frame: " << desired_end_effector_frame << endl;
#endif

				// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
				get_current_kinematic_model()->inverse_kinematics_transform(desired_joints, desired_joints_old, desired_end_effector_frame);

#if(DEBUG_JOINTS)
				std::cerr.precision(15);
				std::cerr << "JOINTS " << desired_joints.transpose() << endl;
#endif
				// Postcondition I - check desired Cartesian position, basing on the upper platform pose.
				get_current_kinematic_model()->check_cartesian_pose(desired_end_effector_frame);
				//get_current_kinematic_model()->check_joints(desired_joints);

				// Transform joints to motors.
				get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new, desired_joints);

				// Postcondition II  - check whether the desired motor position is valid.
				get_current_kinematic_model()->check_motor_position(desired_motor_pos_new);

				// Remember the currently desired joints as old.
				desired_joints_old = desired_joints;
				break;
			}
			default:
				// Throw non-fatal error - invalid pose specification.
				BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_pose_specification());
				break;
		} //: switch (instruction.spkm.set_pose_specification)
	} catch (boost::exception &e_) {
		// TODO add other context informations that are available.
		e_ << mrrocpp::edp::spkm::pose_specification(instruction.spkm.set_pose_specification);

		// Rethrow the catched exception.
		boost::exception_ptr e_ptr = boost::current_exception();
		boost::rethrow_exception(e_ptr);
	}
}

void effector::execute_motion()
{
	DEBUG_METHOD;

	// Note: at this point we assume, that desired_motor_pos_new holds a validated data.
	switch (instruction.spkm.motion_variant)
	{
		case lib::epos::NON_SYNC_TRAPEZOIDAL:
			DEBUG_COMMAND("NON_SYNC_TRAPEZOIDAL");

			// Execute command
			for (size_t i = 0; i < axes.size(); ++i) {
				if (is_synchronised()) {
#if(DEBUG_MOTORS)
					std::cerr << "MOTOR: absolute[" << i << "] ( " << (int) desired_motor_pos_old[i] << "->" << (int) desired_motor_pos_new[i] << ")" << endl;
#endif
					if (!robot_test_mode) {
						// Skip commanding motor if target and last positions and equal.
						if (fabs(desired_motor_pos_new[i] - desired_motor_pos_old[i]) < 1.0)
							continue;
						axes[i]->setProfileVelocity(Vdefault[i]);
						axes[i]->setProfileAcceleration(Adefault[i]);
						axes[i]->setProfileDeceleration(Ddefault[i]);

						// Re-enable the moog motor;
						if(axes[i] == axis2) axes[i]->reset();

						axes[i]->moveAbsolute(desired_motor_pos_new[i]);
					} else {
						current_joints[i] = desired_joints[i];
						current_motor_pos[i] = desired_motor_pos_new[i];
					}
				} else {
#if(DEBUG_MOTORS)
					std::cerr << "MOTOR: relative[" << i << "] ( " << (int) desired_motor_pos_old[i] << "->" << (int) desired_motor_pos_new[i] << ")" << endl;
#endif
					if (!robot_test_mode) {
						if (fabs(desired_motor_pos_new[i]) < 1.0)
							continue;
						std::cerr << " dupa5\n";
						axes[i]->setProfileVelocity(Vdefault[i]);
						axes[i]->setProfileAcceleration(Adefault[i]);
						axes[i]->setProfileDeceleration(Ddefault[i]);

						// Re-enable the moog motor;
						if(axes[i] == axis2) axes[i]->reset();

						axes[i]->moveRelative(desired_motor_pos_new[i]);
					} else {
						current_joints[i] += desired_joints[i];
						current_motor_pos[i] += desired_motor_pos_new[i];
					}
				}
			}
			break;
		case lib::epos::SYNC_TRAPEZOIDAL: {
			DEBUG_COMMAND("SYNC_TRAPEZOIDAL");

			// Motion calculation is done in dimensionless units, but it assumes they are coherent
			// Delta[turns], Vmax[turns per second], Amax[turns per seconds per seconds]
			Matrix <double, 6, 1> Delta, Vmax, Amax, Vnew, Anew, Dnew;

			for (size_t i = 0; i < 6; ++i) {
				Delta[i] = fabs(desired_motor_pos_new[i] - desired_motor_pos_old[i]) / PARAMS.encoder_resolution[i];
				Vmax[i] = Vdefault[i];
				Amax[i] = Adefault[i];
			}

			// Convert to 'rotations' and 'seconds' units.
			Vmax /= maxon::epos::SECONDS_PER_MINUTE;
			Amax /= maxon::epos::SECONDS_PER_MINUTE;

#if(DEBUG_MOTORS)
			for (size_t i = 0; i < number_of_servos; ++i) {
				std::cerr.precision(15);
				std::cerr << "new - old[" << i << "]: " << desired_motor_pos_new[i] << " - " << desired_motor_pos_old[i]
						<< " = " << Delta[i] << endl;
			}
#endif

			// Calculate time of trapezoidal profile motion according to commanded acceleration and velocity limits.
			double t = ppm <6>(Delta, Vmax, Amax, Vnew, Anew, Dnew);

			// Convert back to Maxon-specific units.
			Vnew *= maxon::epos::SECONDS_PER_MINUTE;
			Anew *= maxon::epos::SECONDS_PER_MINUTE;
			Dnew *= maxon::epos::SECONDS_PER_MINUTE;

#if(DEBUG_MOTORS)
			std::cerr.precision(5);
			std::cerr << "Delta:\n" << Delta.transpose() << endl << "Vmax:\n" << Vmax.transpose() << endl << "Amax:\n"
					<< Amax.transpose() << endl << endl;
#endif

			if (t > 0) {
#if(DEBUG_MOTORS)
				std::cerr.precision(5);
				std::cerr << "Vnew:\n" << Vnew.transpose() << endl << "Anew:\n" << Anew.transpose() << endl << "Dnew:\n"
						<< Dnew.transpose() << endl << endl;
#endif
				if (!robot_test_mode) {
					// Motion motion parameters for every controller.
					for (size_t i = 0; i < axes.size(); ++i) {
						// Check delta !=0 - in this case increments are integers, through motions with |deltas| < 1 are ignored.
						if (Delta[i] == 0)
							continue;
						// Set profile mode.
						axes[i]->setOperationMode(maxon::epos::OMD_PROFILE_POSITION_MODE);
						axes[i]->setPositionProfileType(0); // Trapezoidal velocity profile

						// Apply Maxon-specific value limits (zero is not allowed).
						if (Vnew[i] < 1) {
							Vnew[i] = 1;
						}
						if (Vnew[i] > Vdefault[i]) {
							Vnew[i] = Vdefault[i];
						}

						// Apply Maxon-specific value limits (zero is not allowed).
						if (Anew[i] < 1) {
							Anew[i] = 1;
						}
						if (Anew[i] > Ddefault[i]) {
							Anew[i] = Ddefault[i];
						}

						// Apply Maxon-specific value limits (zero is not allowed).
						if (Dnew[i] < 1) {
							Dnew[i] = 1;
						}
						if (Dnew[i] > Ddefault[i]) {
							Dnew[i] = Ddefault[i];
						}

						// Apply trapezoidal profile settings.
						axes[i]->setProfileVelocity(Vnew[i]);
						axes[i]->setProfileAcceleration(Anew[i]);
						axes[i]->setProfileDeceleration(Dnew[i]);

						// Set new motion target
						axes[i]->setTargetPosition(desired_motor_pos_new[i]);

						// Re-enable the moog motor;
						if(axes[i] == axis2) axes[i]->reset();
					}
				}

				// Start motion
				for (size_t i = 0; i < axes.size(); ++i) {
					if (Delta[i] != 0) {
						if (is_synchronised()) {
							// Absolute motion
							if (!robot_test_mode) {
								axes[i]->startAbsoluteMotion();
							} else {
								current_joints[i] = desired_joints[i];
								current_motor_pos[i] = desired_motor_pos_new[i];
							}
						} else {
							// Relative motion
							if (!robot_test_mode) {
								axes[i]->startRelativeMotion();
							} else {
								current_joints[i] += desired_joints[i];
								current_motor_pos[i] += desired_motor_pos_new[i];
							}
						}
					}
				}
			}
		}
			break;
		default:
			// Throw non-fatal error - motion type not supported.
			BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_motion_type());
			break;
	} //: switch (instruction.spkm.motion_variant)
}

void effector::interpolated_motion_in_operational_space()
{
	DEBUG_METHOD;

	if (!is_synchronised())
		// Throw non-fatal error - this mode requires synchronization.
		BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_robot_unsynchronized());

	// Check state of the robot.
	if (controller_state_edp_buf.robot_in_fault_state)
		BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::fe_robot_in_fault_state());

	// Check whether current cartesian pose (in fact the one where the previous motion ended) is known.
	if (!is_current_cartesian_pose_known)
		BOOST_THROW_EXCEPTION(mrrocpp::edp::spkm::nfe_current_cartesian_pose_unknown());

	// Check pose specification.
	if (instruction.spkm.set_pose_specification == lib::spkm::WRIST_XYZ_EULER_ZYZ) {
		DEBUG_COMMAND("WRIST_XYZ_EULER_ZYZ");
		// Retrieve the desired homogeneous matrix on the base of received six  variables - a Euler Z-Y-Z representation.
		desired_end_effector_frame.set_from_xyz_euler_zyz_without_limits(Xyz_Euler_Zyz_vector(instruction.spkm.goal_pos));
	} else if (instruction.spkm.set_pose_specification == lib::spkm::TOOL_XYZ_EULER_ZYZ) {
		DEBUG_COMMAND("TOOL_XYZ_EULER_ZYZ");
		// Retrieve the desired homogeneous matrix on the base of received six  variables - a Euler Z-Y-Z representation.
		desired_spkm_frame.set_from_xyz_euler_zyz_without_limits(Xyz_Euler_Zyz_vector(instruction.spkm.goal_pos));
//		desired_spkm_frame.set_from_xyz_rpy(lib::Xyz_Rpy_vector(instruction.spkm.goal_pos));
		// Transform to the wrist frame.
		desired_end_effector_frame = desired_spkm_frame * !shead_frame;
#if(DEBUG_FRAMES)
		std::cerr << "desired_spkm_frame: " << desired_spkm_frame << endl;
#endif
	} else
		// Other pose specifications aren't valid in this type of movement.
		BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_command());

#if(DEBUG_FRAMES)
	std::cerr << "Wrist frame: " << desired_end_effector_frame << endl;
#endif
	// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
	get_current_kinematic_model()->inverse_kinematics_transform(desired_joints, desired_joints_old, desired_end_effector_frame);

	// Postcondition I - check desired Cartesian position, basing on the upper platform pose.
	get_current_kinematic_model()->check_cartesian_pose(desired_end_effector_frame);

	// Transform joints to motors.
	get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new, desired_joints);

	// Postcondition II  - check whether the desired motor position is valid.
	get_current_kinematic_model()->check_motor_position(desired_motor_pos_new);

	if (!robot_test_mode) {
		// Check whether robot is standing still.
		for (size_t i = 0; i < axes.size(); ++i) {
			if (!axes[i]->isTargetReached())
				BOOST_THROW_EXCEPTION(mrrocpp::edp::spkm::nfe_motion_in_progress());
		}
	}

	// Calculate time - currently the motion time is set to 5s.
	// TODO: analyze required (desired) movement time -> III cases: t<t_req, t=t_req, t>t_req.
	double motion_time = instruction.spkm.estimated_time;

	// Constant time for one segment - 250ms.
	//double segment_time = 1;//0.25;

	// Divide motion time into segments (time slices).
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, 1> time_invervals;
	divide_motion_time_into_constant_time_deltas <lib::spkm::NUM_OF_MOTION_SEGMENTS>(time_invervals, motion_time);

	// Check time intervals.
	check_time_distances <lib::spkm::NUM_OF_MOTION_SEGMENTS>(time_invervals);

	// Interpolate motor poses - equal to number of segments +1 (the start pose).
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS + 1, lib::spkm::NUM_OF_SERVOS> motor_interpolations;

	// Check pose specification.
	if (instruction.spkm.set_pose_specification == lib::spkm::WRIST_XYZ_EULER_ZYZ) {
		// Perform motion in wrist frame.
		cubic_polynomial_interpolate_motor_poses <lib::spkm::NUM_OF_MOTION_SEGMENTS + 1, lib::spkm::NUM_OF_SERVOS>(motor_interpolations, motion_time, time_invervals, get_current_kinematic_model(), desired_joints_old, current_end_effector_frame, desired_end_effector_frame);
	} else if (instruction.spkm.set_pose_specification == lib::spkm::TOOL_XYZ_EULER_ZYZ) {
		// Perform motion in tool frame.
		cubic_polynomial_interpolate_motor_poses_in_tool_frame <lib::spkm::NUM_OF_MOTION_SEGMENTS + 1,
				lib::spkm::NUM_OF_SERVOS>(motor_interpolations, motion_time, time_invervals, get_current_kinematic_model(), desired_joints_old, current_spkm_frame, desired_spkm_frame, shead_frame);
	} else
		// Other pose specifications aren't valid in this type of movement.
		BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_command());

	// Compute motor_deltas for segments.
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_deltas_for_segments;
	compute_motor_deltas_for_segments <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS>(motor_deltas_for_segments, motor_interpolations);

	// Compute tau coefficient matrix of the (1.48) equation.
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_MOTION_SEGMENTS> tau_coefficients;
	compute_tau_coefficients_matrix <lib::spkm::NUM_OF_MOTION_SEGMENTS>(tau_coefficients, time_invervals);

	// Compute right side vector of the (1.48) equation - for all motors!!
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> right_side_coefficients;
	compute_right_side_coefficients_vector <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS>(right_side_coefficients, motor_deltas_for_segments, time_invervals);

	// Compute 2w polynomial coefficients for all motors!!
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_2w;
	compute_motor_2w_polynomial_coefficients <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS>(motor_2w, tau_coefficients, right_side_coefficients);

	// Compute 1w polynomial coefficients for all motors!!
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_1w;
	compute_motor_1w_polynomial_coefficients <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS>(motor_1w, motor_2w, motor_deltas_for_segments, time_invervals);

	// Compute 3w polynomial coefficients for all motors!!
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_3w;
	compute_motor_3w_polynomial_coefficients <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS>(motor_3w, motor_2w, motor_deltas_for_segments, time_invervals);

	// Compute 0w polynomial coefficients for all motors!!
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_0w;
	compute_motor_0w_polynomial_coefficients <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS>(motor_0w, motor_interpolations);

#if(DEBUG_PVT)
	std::cerr << "time_deltas = [ \n" << time_invervals << "\n ]; \n";
	std::cerr << "m0w = [\n" << motor_0w << "\n ]; \n";
	std::cerr << "m1w = [\n" << motor_1w << "\n ]; \n";
	std::cerr << "m2w = [\n" << motor_2w << "\n ]; \n";
	std::cerr << "m3w = [\n" << motor_3w << "\n ]; \n";
#endif

	// Recalculate extreme velocities taking into consideration required units
	// (Vdefault is given in [rpm], and on the base of w0..3 coefficients we can compute v in [turns per second])
	double vmin[lib::spkm::NUM_OF_SERVOS];
	double vmax[lib::spkm::NUM_OF_SERVOS];
	for (size_t mtr = 0; mtr < lib::spkm::NUM_OF_SERVOS; ++mtr) {
		vmin[mtr] = (-1.0) * MotorVmax[mtr] * PARAMS.encoder_resolution[mtr] / 60.0;
		vmax[mtr] = MotorVmax[mtr] * PARAMS.encoder_resolution[mtr] / 60.0;
	}
	// Check extreme velocities for all segments and motors.
	check_velocities <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS>(vmin, vmax, motor_3w, motor_2w, motor_1w);

	// Recalculate extreme accelerations taking into consideration required units
	// (A- and Ddefault are given in [rpm/s], and on the base of w0..3 coefficients we can compute A and D in [turns per second^2])
	double amin[lib::spkm::NUM_OF_SERVOS];
	double amax[lib::spkm::NUM_OF_SERVOS];
	for (size_t mtr = 0; mtr < lib::spkm::NUM_OF_SERVOS; ++mtr) {
		amin[mtr] = (-1.0) * MotorAmax[mtr] * PARAMS.encoder_resolution[mtr] / 60.0;
		amax[mtr] = MotorAmax[mtr] * PARAMS.encoder_resolution[mtr] / 60.0;
	}
	// Check extreme velocities for all segments and motors.
	check_accelerations <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS>(amin, amax, motor_3w, motor_2w, time_invervals);

	// Compute PVT triplets for generated segments (thus n+1 points).
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS + 1, lib::spkm::NUM_OF_SERVOS> p;
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS + 1, lib::spkm::NUM_OF_SERVOS> v;
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS + 1, 1> t;
	compute_pvt_triplets_for_epos <lib::spkm::NUM_OF_MOTION_SEGMENTS + 1, lib::spkm::NUM_OF_SERVOS>(p, v, t, time_invervals, motor_3w, motor_2w, motor_1w, motor_0w);

#if(DEBUG_PVT)
	std::cerr << "p = [ \n" << p << "\n ]; \n";
	std::cerr << "v = [ \n" << v << "\n ]; \n";
	std::cerr << "t = [ \n" << t << "\n ]; \n";
#endif

	// Recalculate units: p[qc], v[rpm (revolutions per minute) per second], t[miliseconds].0x4101
	for (size_t mtr = 0; mtr < lib::spkm::NUM_OF_SERVOS; ++mtr) {
		for (size_t pnt = 0; pnt < lib::spkm::NUM_OF_MOTION_SEGMENTS + 1; ++pnt) {
			v(pnt, mtr) *= 60.0 / PARAMS.encoder_resolution[mtr];
			// Apply Maxon-specific value limits (zero is not allowed).
			if ((0 < v(pnt, mtr)) && (v(pnt, mtr) < 1)) {
				v(pnt, mtr) = 1;
			}
			if ((-1 < v(pnt, mtr)) && (v(pnt, mtr) < 0)) {
				v(pnt, mtr) = -1;
			}
			/*			if (v(pnt, mtr) > Vdefault[mtr]) {
			 v(pnt, mtr) = Vdefault[mtr];
			 }*/
		}
		//p.transpose().row(mtr) /= PARAMS.encoder_resolution[mtr];
		/*							v.transpose().row(mtr) = v.transpose().row(mtr) * epos::epos::SECONDS_PER_MINUTE /
		 PARAMS.encoder_resolution[mtr];*/
	}
	// Recalculate time to [ms].
	t *= 1000;

#if(DEBUG_PVT)
	std::cerr << " !Values after units recalculations!\n";
	std::cerr << "p = [ \n" << p << "\n ]; \n";
	std::cerr << "v = [ \n" << v << "\n ]; \n";
	std::cerr << "t = [ \n" << t << "\n ]; \n";
#endif

#if(DEBUG_PVT)
	struct timeval tv;
	std::string dir;
	// Get time of day.
	if (gettimeofday(&tv, NULL) == -1) {
		perror("gettimeofday()");
	} else {
		// Create unique directory.
		dir = "/home/tkornuta/pkm_measures/5_" + boost::lexical_cast <std::string>(tv.tv_sec);
		if (mkdir(dir.c_str(), 0777) == -1) {
			perror("mkdir()");
			dir += "_";
		} else
			dir += "/";
		// Generate unique name.
		std::string filename = dir + "description.txt";
		ofstream descfile;
		descfile.open(filename.c_str());

		// Write motion description.
		// All values were previously computed in switch (instruction.spkm.variant) - the lib::spkm::FRAME case.

		// Motion time and number of interpolation points.
		descfile << "Motion time: " << motion_time << endl;
		descfile << "Number of segments: " << lib::spkm::NUM_OF_MOTION_SEGMENTS << endl;

		// Cartesian poses.
		descfile << "Current (assumed) end-effector pose:\n" << current_end_effector_frame << endl;
		descfile << "Desired end-effector pose:\n" << desired_end_effector_frame << endl;
		// Joints.
		descfile << "Current (assumed) joints:\t" << desired_joints_old.transpose() << endl;
		descfile << "Desired joints:\t\t\t" << desired_joints.transpose() << endl;
		// Motors.
		descfile << "Current (assumed) motors:\t" << current_motor_pos.transpose() << endl;
		descfile << "Desired motors:\t\t\t" << desired_motor_pos_new.transpose() << endl;

		// Check which axis is going to be moved.
		for (size_t i = 0; i < axes.size(); ++i) {
			descfile << "Axis " << i << ": "
					<< ((p(0, i) != p(lib::spkm::NUM_OF_MOTION_SEGMENTS, i)) ? "moving" : "not moving")<< endl;
				}

		descfile.close();
		std::cerr << "Motion description was written to file: " << filename << endl;

		// Write motion description to files.
		// For every axis six files are created:
		// - one containing start and stop points.
		// - one containing list of PVT triplets.
		// - one containing trajectory parameters (m0, ..., m3) for every segment.
		for (size_t i = 0; i < axes.size(); ++i) {
			// Start and stop points.
			// Generate unique name.
			std::string filename = dir + "axis" + boost::lexical_cast <std::string>(i) + "_start_stop.csv";
			ofstream axis_start_stop;
			axis_start_stop.open(filename.c_str());
			// Write start and stop positions.
			axis_start_stop << (int) current_motor_pos(i) << "\r\n";
			axis_start_stop << (int) desired_motor_pos_new(i) << "\r\n";
			// Close file for given axis motion.
			axis_start_stop.close();

			// List of PVT triplets.
			// Generate unique name.
			filename = dir + "axis" + boost::lexical_cast <std::string>(i) + "_pvt.csv";
			ofstream axis_pvt;
			axis_pvt.open(filename.c_str());
			// Write header.
			axis_pvt << "qc;rpm;ms;\r\n";
			// Write triplets.
			for (size_t pnt = 0; pnt < lib::spkm::NUM_OF_MOTION_SEGMENTS + 1; ++pnt) {
				axis_pvt << (int) p(pnt, i) << ";" << (int) v(pnt, i) << ";" << (int) t(pnt) << ";\r\n";
			} //: for points
			  // Close file for given axis.
			axis_pvt.close();
			std::cerr << "PVT for axis " << i << " were written to file: " << filename << endl;

			// List of trajectory parameters for every segment.
			// Generate unique name.
			filename = dir + "axis" + boost::lexical_cast <std::string>(i) + "_m0123.csv";
			ofstream axis_m0123;
			axis_m0123.open(filename.c_str());
			// Write header.
			axis_m0123 << "m0w;m1w;m2w;m3w;\r\n";
			// Write parameters.
			for (size_t sgt = 0; sgt < lib::spkm::NUM_OF_MOTION_SEGMENTS; ++sgt) {
				axis_m0123 << motor_0w(sgt, i) << ";" << motor_1w(sgt, i) << ";" << motor_2w(sgt, i) << ";"
						<< motor_3w(sgt, i) << ";\r\n";
			} //: for segments
			  // Close file for given axis.
			axis_m0123.close();
			std::cerr << "Trajectory parameters for axis " << i << " were written to file: " << filename << endl;

			// Write
		} //: for axes
	} //: else
#endif

	// Check which motor moves.
	Eigen::Matrix <bool, 1, lib::spkm::NUM_OF_SERVOS> change;
	check_pvt_translocation <lib::spkm::NUM_OF_MOTION_SEGMENTS + 1, lib::spkm::NUM_OF_SERVOS>(p, change);

	// Execute motion
	if (!robot_test_mode) {
		// Reset the Moog motor to disable brake.
		axis2->reset();

		// Setup motion parameters
		for (size_t i = 0; i < axes.size(); ++i) {

			// If no translocation is required for given axis - skip the motion (in order to save time).
			if (!change(i))
				continue;
#if(DEBUG_PVT)
			std::cerr << "Axis " << i << " position change: setting parameters. \n";
#endif

			// Setup motion parameters.
			axes[i]->setOperationMode(maxon::epos::OMD_INTERPOLATED_POSITION_MODE);
			axes[i]->setProfileVelocity(MotorVmax[i]);
			axes[i]->setProfileAcceleration(MotorAmax[i]);
			axes[i]->setProfileDeceleration(MotorAmax[i]);
			axes[i]->clearPvtBuffer();
			// TODO: setup acceleration and velocity limit values
			axes[i]->clearPvtBuffer();
			for (size_t pnt = 0; pnt < lib::spkm::NUM_OF_MOTION_SEGMENTS + 1; ++pnt) {
				axes[i]->setInterpolationDataRecord((int32_t) p(pnt, i), (int32_t) v(pnt, i), (uint8_t) t(pnt));
#if(DEBUG_PVT)
				printf("\rsend: %zd/%zd, free: %2d", pnt, i, axes[i]->getActualBufferSize());
				fflush(stdout);
#endif
			}
#if(DEBUG_PVT)
			printf("\n");
#endif

			const maxon::UNSIGNED16 uploaded = axes[i]->getInterpolationBufferPosition();
			if (uploaded != lib::spkm::NUM_OF_MOTION_SEGMENTS + 1) {
				printf("InterpolationBufferPosition for axis %zu: %u\n", i, uploaded);
				BOOST_THROW_EXCEPTION(mrrocpp::edp::epos::nfe_epos_interpolation_buffer()<<motor_number(i));
			}

			const maxon::UNSIGNED16 status = axes[i]->getInterpolationBufferStatus();

			if (axes[i]->checkInterpolationBufferWarning(status)) {
				axes[i]->printInterpolationBufferStatus(status);
			}

			if (axes[i]->checkInterpolationBufferError(status)) {
				printf("InterpolationBufferStatus for axis %zu: 0x%04X\n", i, status);
				BOOST_THROW_EXCEPTION(mrrocpp::edp::epos::nfe_epos_interpolation_buffer()<<motor_number(i));
			}
		}
	} else {
#if(DEBUG_PVT)
		// Display axes movement.
		for (size_t i = 0; i < axes.size(); ++i) {
			std::cerr << "Axis " << i << ": qc;rpm;ms;\r\n";
			for (size_t pnt = 0; pnt < lib::spkm::NUM_OF_MOTION_SEGMENTS + 1; ++pnt) {
				std::cerr << (int) p(pnt, i) << ";" << (int) v(pnt, i) << ";" << (int) t(pnt) << ";\r\n";
			} //: for segments
		} //: for axes
#endif
	} //: end robot_test_mode

	// Start motion
	for (size_t i = 0; i < axes.size(); ++i) {
		// If no translocation is required for given axis - skip the motion (in order to save time).
		if (!change(i))
			continue;

		if (!robot_test_mode) {
			// FIXME: this motion type should be initiated with a CAN broadcast message
			axes[i]->startInterpolatedPositionMotion();
		} else {
			current_joints[i] = desired_joints[i];
			current_motor_pos[i] = desired_motor_pos_new[i];
		}
	}
	// Remember the currently desired joints as old.
	desired_joints_old = desired_joints;
}

void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction_)
{
	DEBUG_METHOD;

	try {
		// Check controller state.
		check_controller_state();

		// we do not check the arm position when only lib::SET is set
		if (instruction.instruction_type != lib::SET) {
			switch (instruction.spkm.get_pose_specification)
			{
				case lib::spkm::MOTOR: {
					DEBUG_COMMAND("MOTOR");
					for (size_t i = 0; i < axes.size(); ++i) {
						if (robot_test_mode) {
							reply.spkm.epos_controller[i].position = current_motor_pos[i];
							reply.spkm.epos_controller[i].current = 0;
							reply.spkm.epos_controller[i].motion_in_progress = false;
						} else {
							current_motor_pos[i] = axes[i]->getActualPosition();
							reply.spkm.epos_controller[i].position = current_motor_pos[i];
							reply.spkm.epos_controller[i].current = axes[i]->getActualCurrent();
							reply.spkm.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
						}
					}
				}
					break;
				case lib::spkm::JOINT: {
					DEBUG_COMMAND("JOINT");
					// Read actual values from the hardware.
					if (!robot_test_mode) {
						for (size_t i = 0; i < axes.size(); ++i) {
							current_motor_pos[i] = axes[i]->getActualPosition();
							reply.spkm.epos_controller[i].current = axes[i]->getActualCurrent();
							reply.spkm.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
						}
					}

					// Do the calculation
					get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

					// Fill the values into a buffer
					for (size_t i = 0; i < number_of_servos; ++i) {
						reply.spkm.epos_controller[i].position = current_joints[i];
					}
				}
					break;
				case lib::spkm::WRIST_XYZ_EULER_ZYZ: {
					DEBUG_COMMAND("WRIST_XYZ_EULER_ZYZ");
					// Return current end-effector pose if it is known (last motion was performed in the cartesian space).
					if (!is_current_cartesian_pose_known)
						current_end_effector_frame.setIdentity();

					Xyz_Euler_Zyz_vector zyz;
					current_end_effector_frame.get_xyz_euler_zyz_without_limits(zyz, current_joints[3], current_joints[4], current_joints[5]);
					zyz.to_table(reply.spkm.current_pose);

#if(DEBUG_FRAMES)
					std::cerr << "Returned WRIST_XYZ_EULER_ZYZ: " << zyz.transpose() << endl;
#endif

					// Return additional informations regarding current and motion.
					if (!robot_test_mode) {
						for (size_t i = 0; i < axes.size(); ++i) {
							reply.spkm.epos_controller[i].current = axes[i]->getActualCurrent();
							reply.spkm.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
						}
					}
				}
					break;
				case lib::spkm::TOOL_XYZ_EULER_ZYZ: {
					DEBUG_COMMAND("TOOL_XYZ_EULER_ZYZ");
					// Return current end-effector pose if it is known (last motion was performed in the cartesian space).
					if (!is_current_cartesian_pose_known)
						current_spkm_frame.setIdentity();

					Xyz_Euler_Zyz_vector zyz;
					current_spkm_frame.get_xyz_euler_zyz(zyz);
					zyz.to_table(reply.spkm.current_pose);
					/*					lib::Xyz_Rpy_vector rpy;
					 current_spkm_frame.get_xyz_rpy(rpy);
					 rpy.to_table(reply.spkm.current_pose);*/

					/*					// Return current end-effector pose if it is known (last motion was performed in the cartesian space).
					 if (is_current_cartesian_pose_known)
					 reply.spkm.current_pose = current_spkm_frame;
					 else
					 // Return identity.
					 reply.spkm.current_pose.setIdentity();*/

#if(DEBUG_FRAMES)
					std::cerr << "Returned TOOL_XYZ_EULER_ZYZ: " << zyz.transpose() << endl;
#endif

					// Return additional informations regarding current and motion.
					if (!robot_test_mode) {
						for (size_t i = 0; i < axes.size(); ++i) {
							reply.spkm.epos_controller[i].current = axes[i]->getActualCurrent();
							reply.spkm.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
						}
					}
				}
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
/*                           Utility routines                               */
/*--------------------------------------------------------------------------*/
void effector::create_threads()
{
	rb_obj = (boost::shared_ptr <common::reader_buffer>) new common::reader_buffer(*this);
	//vis_obj = (boost::shared_ptr <common::vis_server>) new common::vis_server(*this);
}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	manip_effector::single_thread_master_order(nm_task, nm_tryb);
}

lib::INSTRUCTION_TYPE effector::variant_receive_instruction()
{
	return receive_instruction(instruction);
}

void effector::variant_reply_to_instruction()
{
	reply_to_instruction(reply);
}

} // namespace spkm
} // namespace edp
} // namespace mrrocpp
