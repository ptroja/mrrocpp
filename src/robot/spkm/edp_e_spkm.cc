#include <iostream>
#include <fstream>
#include <cstdio>
#include <sys/stat.h>
//<sys/types.h>
#include <boost/foreach.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

#include "edp_e_spkm.h"
#include "base/edp/reader.h"
//#include "base/edp/vis_server.h"

#include "kinematic_model_spkm.h"
#include "kinematic_parameters_spkm.h"
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

using namespace mrrocpp::lib;
using namespace mrrocpp::lib::pvat;
using namespace std;

//! Prints retrieved commands.
#define display_commands 1

//! Prints reference frames.
#define display_frames 1

const uint32_t effector::Vdefault[lib::spkm::NUM_OF_SERVOS] = { 5000UL, 5000UL, 5000UL, 5000UL, 5000UL, 5000UL };
const uint32_t effector::Adefault[lib::spkm::NUM_OF_SERVOS] = { 30000UL, 30000UL, 30000UL, 30000UL, 15000UL, 30000UL };
const uint32_t effector::Ddefault[lib::spkm::NUM_OF_SERVOS] = { 30000UL, 30000UL, 30000UL, 30000UL, 15000UL, 30000UL };

const uint32_t effector::MotorVmax[lib::spkm::NUM_OF_SERVOS] = { 5000UL, 5000UL, 5000UL, 5000UL, 5000UL, 5000UL };
const uint32_t effector::MotorAmax[lib::spkm::NUM_OF_SERVOS] = { 30000UL, 30000UL, 30000UL, 30000UL, 15000UL, 30000UL };

effector::effector(common::shell &_shell, lib::robot_name_t l_robot_name) :
	manip_effector(_shell, l_robot_name)
{
	// Set number of servos.
	number_of_servos = lib::spkm::NUM_OF_SERVOS;

	// Create all kinematic models for SPKM.
	create_kinematic_models_for_given_robot();

	// Zero motors and set initial joint values.
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
		axisA = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 5);
		axisB = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 4);
		axisC = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 6);
		axis1 = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 3);
		axis2 = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 2);
		axis3 = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 1);

		// Collect axes into common array container.
		axes[0] = &(*axisA);
		axesNames[0] = "A";
		axes[1] = &(*axisB);
		axesNames[1] = "B";
		axes[2] = &(*axisC);
		axesNames[2] = "C";
		axes[3] = &(*axis1);
		axesNames[3] = "1";
		axes[4] = &(*axis2);
		axesNames[4] = "2";
		axes[5] = &(*axis3);
		axesNames[5] = "3";

		// Setup the axis array for the IPM handler
		{
			boost::unique_lock <boost::mutex> lock(ipm_handler.mtx);
			ipm_handler.axes = this->axes;
		}
	}
}

void effector::reset_variables()
{
	// Zero all variables related to motor positions.
	for (int i = 0; i < number_of_servos; ++i) {
		current_motor_pos[i] = 0;
	}
	//current_motor_pos << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	desired_motor_pos_old = current_motor_pos;
	desired_motor_pos_new = current_motor_pos;

	// Compute current motor positions on the base of zeroed motors.
	get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
	desired_joints = current_joints;
	desired_joints_old = current_joints;

	// Reset cartesian-related variables.
	current_end_effector_frame.setIdentity();
	desired_end_effector_frame.setIdentity();

	is_previous_cartesian_pose_known = false;

	// Read tool (SHEAD) transformation from configuration file.

	if (config.exists("shead_frame")) {
		// Try to read the shead_frame from ini file.
		try {
			shead_frame.set(config.value <std::string> ("shead_frame"));
		} catch (std::exception& e_) {
			// Print failure reason.
			cout << e_.what() << endl;
			// Set identity.
			shead_frame.setIdentity();
		}
	} else
		// set identity by default.
		shead_frame.setIdentity();
	cout << "shead_frame: [" << shead_frame << "]\n";

}

void effector::check_controller_state()
{
	if (robot_test_mode)
		return;

	// Try to get state of each axis
	unsigned int referenced = 0;
	unsigned int powerOn = 0;
	unsigned int enabled = 0;
	// Check axes.
	for (size_t i = 0; i < axes.size(); ++i) {
		try {
			// Get current epos state.
			maxon::epos::actual_state_t state = axes[i]->getState();
			if (state != maxon::epos::OPERATION_ENABLE) {
				cout << string("Axis ") << axesNames[i] << endl;
				// Print state.
				axes[i]->printState();
				// Check if in the FAULT state
				if (state == maxon::epos::FAULT) {
					// Read number of errors
					int errNum = axes[i]->getNumberOfErrors();
					for (int j = 1; j <= errNum; ++j) {
						// Get the detailed error
						uint32_t errCode = axes[i]->getErrorHistory(j);
						// Send message to SR.
						msg->message(mrrocpp::lib::FATAL_ERROR, string("Axis ") + axesNames[i] + ": "
								+ axes[i]->ErrorCodeMessage(errCode));
					}
				} else if (state == maxon::epos::SWITCH_ON_DISABLED) {
					// Send message to SR.
					msg->message(mrrocpp::lib::FATAL_ERROR, string("Epos controlling ") + axesNames[i]
							+ " rotation is disabled");
				} //: if fault || disabled
			} else {
				// EPOS in enabled state.
				enabled++;
			}
			if (axes[i]->isReferenced()) {
				// Do not break from this loop so this is a also a preliminary axis error check
				referenced++;
			}
			powerOn++;
		} catch (...) {
			// Probably the axis is not powered on, do nothing.
		}
	}
	// Robot is synchronized if all axes are referenced
	controller_state_edp_buf.is_synchronised = (referenced == axes.size());
	// Check whether all axes are powered.
	controller_state_edp_buf.is_power_on = (powerOn == axes.size());
	// Check fault state.
	controller_state_edp_buf.robot_in_fault_state = (enabled != axes.size());
}

void effector::get_controller_state(lib::c_buffer &instruction)
{
	try {
		// False is the initial value
		controller_state_edp_buf.is_synchronised = false;
		controller_state_edp_buf.is_power_on = false;
		controller_state_edp_buf.robot_in_fault_state = false;

		// Check controller state.
		check_controller_state();

		// Copy data to reply buffer
		reply.controller_state = controller_state_edp_buf;

		// Check if it is safe to calculate joint positions
		if (is_synchronised()) {
			get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
		}

		// Lock data structure during update
		{
			boost::mutex::scoped_lock lock(effector_mutex);

			// Initialize internal data
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

void effector::synchronise(void)
{

	try {

		if (robot_test_mode) {
			controller_state_edp_buf.is_synchronised = true;

			return;
		}

		// switch to homing mode
		BOOST_FOREACH(maxon::epos * node, axes)
					{
						node->setOperationMode(maxon::epos::OMD_HOMING_MODE);
					}

		// reset controller
		BOOST_FOREACH(maxon::epos * node, axes)
					{
						node->reset();
					}

		// Do homing with preconfigured parameters
		BOOST_FOREACH(maxon::epos * node, axes)
					{
						node->startHoming();
					}

		// Loop until homing is finished
		bool finished;
		do {
			finished = true;
			BOOST_FOREACH(maxon::epos * node, axes)
						{
							if (!node->isHomingFinished()) {
								finished = false;
							}
						}
		} while (!finished);

		// Hardcoded safety values.
		// TODO: move to configuration file?
		for (size_t i = 0; i < axes.size(); ++i) {
			axes[i]->setMinimalPositionLimit(kinematics::spkm::kinematic_parameters_spkm::lower_motor_pos_limits[i]
					- 100);
			axes[i]->setMaximalPositionLimit(kinematics::spkm::kinematic_parameters_spkm::upper_motor_pos_limits[i]
					+ 100);
		}

		// Move the longest linear axis to the 'zero' position with a fast motion command
		/*	axisB->writeProfileVelocity(5000UL);
		 axisB->writeProfileAcceleration(1000UL);
		 axisB->writeProfileDeceleration(1000UL);
		 axisB->moveAbsolute(-57500);*/

		// Reset internal state of the motor positions
		for (int i = 0; i < number_of_servos; ++i) {
			current_motor_pos[i] = desired_motor_pos_old[i] = 0;
		}

		// Compute joints positions in the home position
		get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

		// Now the robot is synchronised
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

void effector::move_arm(const lib::c_buffer &instruction)
{

	try {
		// Check command type.
		switch (ecp_edp_cbuffer.variant)
		{
			case lib::spkm::POSE:
				// Special case: operational motion.
				if (ecp_edp_cbuffer.motion_variant == lib::epos::OPERATIONAL) {
					interpolated_motion_in_operational_space();
					break;
				}

				// Parse command.
				parse_motor_command();

				// Execute motion.
				execute_motor_motion();

				break;
			case lib::spkm::QUICKSTOP:
				if (!robot_test_mode) {
					// Execute command
					BOOST_FOREACH(maxon::epos * node, axes)
								{
									// Brake with Quickstop command
									node->setState(maxon::epos::QUICKSTOP);
								}
				}
				// Internal position counters need not be updated
				return;
			case lib::spkm::CLEAR_FAULT:
				BOOST_FOREACH(maxon::epos * node, axes)
							{
								node->printState();

								// Check if in a FAULT state
								if (node->getState() == 11) {
									maxon::UNSIGNED8 errNum = node->getNumberOfErrors();
									cerr << "readNumberOfErrors() = " << (int) errNum << endl;
									for (maxon::UNSIGNED8 i = 1; i <= errNum; ++i) {

										maxon::UNSIGNED32 errCode = node->getErrorHistory(i);

										cerr << node->ErrorCodeMessage(errCode) << endl;
									}
									if (errNum > 0) {
										node->clearNumberOfErrors();
									}
									node->setState(maxon::epos::FAULT_RESET);
								}

								// Reset node.
								node->reset();
							}
				// Internal position counters need not be updated
				return;
			default:
				break;
		}

		/*		for (int i = 0; i < 6; ++i) {
		 cout << "OLD     MOTOR[" << i << "]: " << desired_motor_pos_old[i] << endl;
		 cout << "CURRENT MOTOR[" << i << "]: " << current_motor_pos[i] << endl;
		 cout << "CURRENT JOINT[" << i << "]: " << current_joints[i] << endl;
		 }*/

		// Hold the issued command.
		desired_motor_pos_old = desired_motor_pos_new;

		is_previous_cartesian_pose_known = false;
		// Check whether the motion was performed in the cartesian space - then we know where manipulator will be when the next command arrives:).
		if ((ecp_edp_cbuffer.set_pose_specification == lib::spkm::XYZ_EULER_ZYZ)
				|| (ecp_edp_cbuffer.set_pose_specification == lib::spkm::WRIST_ORIENTED_XYZ_EULER_ZYZ_WITH_TOOL)) {
			// Command was given in the wrist frame.
			current_end_effector_frame = desired_end_effector_frame;
			current_shead_frame = current_end_effector_frame * shead_frame;
			is_previous_cartesian_pose_known = true;
		} else if (ecp_edp_cbuffer.set_pose_specification == lib::spkm::XYZ_EULER_ZYZ) {
			// Command was given in the tool (SHEAD) frame.
			current_shead_frame = desired_shead_frame;
			current_end_effector_frame = desired_end_effector_frame * !shead_frame;
			is_previous_cartesian_pose_known = true;
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
	try {
		switch (ecp_edp_cbuffer.set_pose_specification)
		{
			case lib::spkm::MOTOR: {
				// Copy data directly from buffer
				for (int i = 0; i < number_of_servos; ++i) {
					desired_motor_pos_new[i] = ecp_edp_cbuffer.motor_pos[i];
#ifdef display_commands
					cout << "MOTOR[ " << i << "]: " << desired_motor_pos_new[i] << endl;
#endif
				}

				if (is_synchronised()) {
					// Check the desired motor (only motors!) values if they are absolute.
					get_current_kinematic_model()->check_motor_position(desired_motor_pos_new);
				}

				break;
			}
			case lib::spkm::JOINT: {
				// Copy data directly from buffer
				for (int i = 0; i < number_of_servos; ++i) {
					desired_joints[i] = ecp_edp_cbuffer.joint_pos[i];
#ifdef display_commands
					cout << "JOINT[ " << i << "]: " << desired_joints[i] << endl;
#endif
				}

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
			}
			case lib::spkm::WRIST_ORIENTED_XYZ_EULER_ZYZ_WITH_TOOL:
				// In case of SYNC_TRAPEZOIDAL and NON_SYNC_TRAPEZOIDAL those two types of commands are executed in exactly the same way.
			case lib::spkm::XYZ_EULER_ZYZ: {
#ifdef display_commands
				cout << "XYZ_EULER_ZYZ: [";
				for (unsigned int i = 0; i < 6; ++i) {
					cout << ecp_edp_cbuffer.goal_pos[i] << ", ";
				}
				cout << "]\n";
#endif

				if (!is_synchronised())
					// Throw non-fatal error - this mode requires synchronization.
					BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_robot_unsynchronized());

				// Retrieve the desired homogeneous matrix on the base of received six  variables - a Euler Z-Y-Z representation.
				desired_end_effector_frame.set_from_xyz_euler_zyz_without_limits(Xyz_Euler_Zyz_vector(ecp_edp_cbuffer.goal_pos));
#ifdef display_frames
				cout << desired_end_effector_frame << endl;
#endif
				// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
				get_current_kinematic_model()->inverse_kinematics_transform(desired_joints, desired_joints_old, desired_end_effector_frame);

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
			case lib::spkm::TOOL_ORIENTED_XYZ_EULER_ZYZ_WITH_TOOL: {
#ifdef display_commands
				cout << "TOOL_ORIENTED_XYZ_EULER_ZYZ_WITH_TOOL: [";
				for (unsigned int i = 0; i < 6; ++i) {
					cout << ecp_edp_cbuffer.goal_pos[i] << ", ";
				}
				cout << "]\n";
#endif
				if (!is_synchronised())
					// Throw non-fatal error - this mode requires synchronization.
					BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_robot_unsynchronized());

				// Retrieve the desired homogeneous matrix on the base of received six  variables - a Euler Z-Y-Z representation.
				desired_shead_frame.set_from_xyz_euler_zyz_without_limits(Xyz_Euler_Zyz_vector(ecp_edp_cbuffer.goal_pos));
#ifdef display_frames
				cout << "Tool frame: " << desired_shead_frame << endl;
#endif
				// Transform to the wrist frame.
				desired_end_effector_frame = desired_shead_frame * !shead_frame;
#ifdef display_frames
				cout << "Wrist frame: " << desired_end_effector_frame << endl;
#endif

				// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
				get_current_kinematic_model()->inverse_kinematics_transform(desired_joints, desired_joints_old, desired_end_effector_frame);

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
		} //: switch (ecp_edp_cbuffer.set_pose_specification)
	} catch (boost::exception &e_) {
		// TODO add other context informations that are available.
		e_ << mrrocpp::edp::spkm::pose_specification(ecp_edp_cbuffer.set_pose_specification);

		// Rethrow the catched exception.
		boost::exception_ptr e_ptr = boost::current_exception();
		boost::rethrow_exception(e_ptr);
	}
}

void effector::execute_motor_motion()
{
	// Note: at this point we assume, that desired_motor_pos_new holds a validated data.
	switch (ecp_edp_cbuffer.motion_variant)
	{
		case lib::epos::NON_SYNC_TRAPEZOIDAL:
			// Execute command
			for (size_t i = 0; i < axes.size(); ++i) {
				if (is_synchronised()) {
					cout << "MOTOR: moveAbsolute[" << i << "] ( " << desired_motor_pos_new[i] << ")" << endl;
					if (!robot_test_mode) {
						axes[i]->setProfileVelocity(Vdefault[i]);
						axes[i]->setProfileAcceleration(Adefault[i]);
						axes[i]->setProfileDeceleration(Ddefault[i]);
						axes[i]->moveAbsolute(desired_motor_pos_new[i]);
					} else {
						current_joints[i] = desired_joints[i];
						current_motor_pos[i] = desired_motor_pos_new[i];
					}
				} else {
					cout << "MOTOR: moveRelative[" << i << "] ( " << desired_motor_pos_new[i] << ")" << endl;
					if (!robot_test_mode) {
						axes[i]->setProfileVelocity(Vdefault[i]);
						axes[i]->setProfileAcceleration(Adefault[i]);
						axes[i]->setProfileDeceleration(Ddefault[i]);
						axes[i]->moveRelative(desired_motor_pos_new[i]);
					} else {
						current_joints[i] += desired_joints[i];
						current_motor_pos[i] += desired_motor_pos_new[i];
					}
				}
			}
			break;
		case lib::epos::SYNC_TRAPEZOIDAL: {
			// Motion calculation is done in dimensionless units, but it assumes they are coherent
			// Delta[turns], Vmax[turns per second], Amax[turns per seconds per seconds]
			Matrix <double, 6, 1> Delta, Vmax, Amax, Vnew, Anew, Dnew;

			for (int i = 0; i < 6; ++i) {
				Delta[i] = fabs(desired_motor_pos_new[i] - desired_motor_pos_old[i])
						/ kinematics::spkm::kinematic_parameters_spkm::encoder_resolution[i];
				cout << "new - old[" << i << "]: " << desired_motor_pos_new[i] << " - " << desired_motor_pos_old[i]
						<< " = " << Delta[i] << endl;
				Vmax[i] = Vdefault[i] / maxon::epos::SECONDS_PER_MINUTE;
				Amax[i] = Adefault[i];
			}

			// Calculate time of trapezoidal profile motion according to commanded acceleration and velocity limits
			double t = ppm <6> (Delta, Vmax, Amax, Vnew, Anew, Dnew);

			cerr << "Delta:\n" << Delta << endl << "Vmax:\n" << Vmax << endl << "Amax:\n" << Amax << endl << endl;

			if (t > 0) {
				// debug display
				cerr << "Vnew:\n" << Vnew << endl << "Anew:\n" << Anew << endl << "Dnew:\n" << Dnew << endl << endl;

				if (!robot_test_mode) {
					// Setup motion parameters
					for (size_t i = 0; i < axes.size(); ++i) {
						if (Delta[i] != 0) {
							axes[i]->setOperationMode(maxon::epos::OMD_PROFILE_POSITION_MODE);
							axes[i]->setPositionProfileType(0); // Trapezoidal velocity profile

							// Adjust velocity settings results in than 1 (minimal value accepted by the EPOS2)
							if (Vnew[i] > 0 && Vnew[i] * maxon::epos::SECONDS_PER_MINUTE < 1.0) {
								Vnew[i] = 1.1 / maxon::epos::SECONDS_PER_MINUTE;
							}

							// Adjust acceleration settings which are less than 1 (minimal value accepted by the EPOS2)
							if (Anew[i] > 0 && Anew[i] < 1) {
								Anew[i] = 1;
							}

							// Adjust deceleration settings which are less than 1 (minimal value accepted by the EPOS2)
							if (Dnew[i] > 0 && Dnew[i] < 1) {
								Dnew[i] = 1;
							}

							axes[i]->setProfileVelocity(Vnew[i] * maxon::epos::SECONDS_PER_MINUTE);
							axes[i]->setProfileAcceleration(Anew[i]);
							axes[i]->setProfileDeceleration(Dnew[i]);
							axes[i]->setTargetPosition(desired_motor_pos_new[i]);
						}
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
	} //: switch (ecp_edp_cbuffer.motion_variant)
}

void effector::interpolated_motion_in_operational_space()
{
	if (!is_synchronised())
		// Throw non-fatal error - this mode requires synchronization.
		BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_robot_unsynchronized());

	// Check whether current cartesian pose (in fact the one where the previous motion ended) is known.
	if (!is_previous_cartesian_pose_known)
		BOOST_THROW_EXCEPTION(mrrocpp::edp::spkm::nfe_current_cartesian_pose_unknown());

	// Retrieve the desired homogeneous matrix on the base of received six  variables - a Euler Z-Y-Z representation.
	desired_end_effector_frame.set_from_xyz_euler_zyz_without_limits(Xyz_Euler_Zyz_vector(ecp_edp_cbuffer.goal_pos));
	cout << desired_end_effector_frame << endl;

	// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
	get_current_kinematic_model()->inverse_kinematics_transform(desired_joints, desired_joints_old, desired_end_effector_frame);

	// Postcondition I - check desired Cartesian position, basing on the upper platform pose.
	get_current_kinematic_model()->check_cartesian_pose(desired_end_effector_frame);

	// Transform joints to motors.
	get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new, desired_joints);

	// Postcondition II  - check whether the desired motor position is valid.
	get_current_kinematic_model()->check_motor_position(desired_motor_pos_new);

	// Calculate time - currently the motion time is set to 5s.
	// TODO: analyze required (desired) movement time -> III cases: t<t_req, t=t_req, t>t_req.
	double motion_time = ecp_edp_cbuffer.estimated_time;

	// Constant time for one segment - 250ms.
	//double segment_time = 1;//0.25;

	// Divide motion time into segments (time slices).
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, 1> time_invervals;
	divide_motion_time_into_constant_time_deltas <lib::spkm::NUM_OF_MOTION_SEGMENTS> (time_invervals, motion_time);

	// Check time intervals.
	check_time_distances <lib::spkm::NUM_OF_MOTION_SEGMENTS> (time_invervals);

	// Interpolate motor poses - equal to number of segments +1 (the start pose).
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS + 1, lib::spkm::NUM_OF_SERVOS> motor_interpolations;
	cubic_polynomial_interpolate_motor_poses <lib::spkm::NUM_OF_MOTION_SEGMENTS + 1, lib::spkm::NUM_OF_SERVOS> (motor_interpolations, motion_time, time_invervals, get_current_kinematic_model(), desired_joints_old, current_end_effector_frame, desired_end_effector_frame);

	// Compute motor_deltas for segments.
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_deltas_for_segments;
	compute_motor_deltas_for_segments <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> (motor_deltas_for_segments, motor_interpolations);

	// Compute tau coefficient matrix of the (1.48) equation.
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_MOTION_SEGMENTS> tau_coefficients;
	compute_tau_coefficients_matrix <lib::spkm::NUM_OF_MOTION_SEGMENTS> (tau_coefficients, time_invervals);

	// Compute right side vector of the (1.48) equation - for all motors!!
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> right_side_coefficients;
	compute_right_side_coefficients_vector <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> (right_side_coefficients, motor_deltas_for_segments, time_invervals);

	// Compute 2w polynomial coefficients for all motors!!
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_2w;
	compute_motor_2w_polynomial_coefficients <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> (motor_2w, tau_coefficients, right_side_coefficients);

	// Compute 1w polynomial coefficients for all motors!!
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_1w;
	compute_motor_1w_polynomial_coefficients <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> (motor_1w, motor_2w, motor_deltas_for_segments, time_invervals);

	// Compute 3w polynomial coefficients for all motors!!
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_3w;
	compute_motor_3w_polynomial_coefficients <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> (motor_3w, motor_2w, motor_deltas_for_segments, time_invervals);

	// Compute 0w polynomial coefficients for all motors!!
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_0w;
	compute_motor_0w_polynomial_coefficients <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> (motor_0w, motor_interpolations);

#if 0
	cout << "time_deltas = [ \n" << time_invervals << "\n ]; \n";
	cout << "m0w = [\n" << motor_0w << "\n ]; \n";
	cout << "m1w = [\n" << motor_1w << "\n ]; \n";
	cout << "m2w = [\n" << motor_2w << "\n ]; \n";
	cout << "m3w = [\n" << motor_3w << "\n ]; \n";
#endif

	// Recalculate extreme velocities taking into consideration required units
	// (Vdefault is given in [rpm], and on the base of w0..3 coefficients we can compute v in [turns per second])
	double vmin[lib::spkm::NUM_OF_SERVOS];
	double vmax[lib::spkm::NUM_OF_SERVOS];
	for (int mtr = 0; mtr < lib::spkm::NUM_OF_SERVOS; ++mtr) {
		vmin[mtr] = (-1.0) * MotorVmax[mtr] * kinematics::spkm::kinematic_parameters_spkm::encoder_resolution[mtr]
				/ 60.0;
		vmax[mtr] = MotorVmax[mtr] * kinematics::spkm::kinematic_parameters_spkm::encoder_resolution[mtr] / 60.0;
	}
	// Check extreme velocities for all segments and motors.
	check_velocities <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> (vmin, vmax, motor_3w, motor_2w, motor_1w);

	// Recalculate extreme accelerations taking into consideration required units
	// (A- and Ddefault are given in [rpm/s], and on the base of w0..3 coefficients we can compute A and D in [turns per second^2])
	double amin[lib::spkm::NUM_OF_SERVOS];
	double amax[lib::spkm::NUM_OF_SERVOS];
	for (int mtr = 0; mtr < lib::spkm::NUM_OF_SERVOS; ++mtr) {
		amin[mtr] = (-1.0) * MotorAmax[mtr] * kinematics::spkm::kinematic_parameters_spkm::encoder_resolution[mtr]
				/ 60.0;
		amax[mtr] = MotorAmax[mtr] * kinematics::spkm::kinematic_parameters_spkm::encoder_resolution[mtr] / 60.0;
	}
	// Check extreme velocities for all segments and motors.
	check_accelerations <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> (amin, amax, motor_3w, motor_2w, time_invervals);

	// Compute PVT triplets for generated segments (thus n+1 points).
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS + 1, lib::spkm::NUM_OF_SERVOS> p;
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS + 1, lib::spkm::NUM_OF_SERVOS> v;
	Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS + 1, 1> t;
	compute_pvt_triplets_for_epos <lib::spkm::NUM_OF_MOTION_SEGMENTS + 1, lib::spkm::NUM_OF_SERVOS> (p, v, t, time_invervals, motor_3w, motor_2w, motor_1w, motor_0w);

#if 1
	cout << "p = [ \n" << p << "\n ]; \n";
	cout << "v = [ \n" << v << "\n ]; \n";
	cout << "t = [ \n" << t << "\n ]; \n";
#endif

	// Recalculate units: p[qc], v[rpm (revolutions per minute) per second], t[miliseconds].
	for (int mtr = 0; mtr < lib::spkm::NUM_OF_SERVOS; ++mtr) {
		for (int pnt = 0; pnt < lib::spkm::NUM_OF_MOTION_SEGMENTS + 1; ++pnt) {
			v(pnt, mtr) *= 60.0 / kinematics::spkm::kinematic_parameters_spkm::encoder_resolution[mtr];
		}
		//p.transpose().row(mtr) /= kinematics::spkm::kinematic_parameters_spkm::encoder_resolution[mtr];
		/*							v.transpose().row(mtr) = v.transpose().row(mtr) * epos::epos::SECONDS_PER_MINUTE /
		 kinematics::spkm::kinematic_parameters_spkm::encoder_resolution[mtr];*/
	}
	// Recalculate time to [ms].
	t *= 1000;

#if 1
	cout << " !Values after units recalculations!\n";
	cout << "p = [ \n" << p << "\n ]; \n";
	cout << "v = [ \n" << v << "\n ]; \n";
	cout << "t = [ \n" << t << "\n ]; \n";
#endif

#if 0
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
		// All values were previously computed in switch (ecp_edp_cbuffer.variant) - the lib::spkm::FRAME case.

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
			<< ((p(0, i) != p(lib::spkm::NUM_OF_MOTION_SEGMENTS, i)) ? "moving" : "not moving") << endl;
		}

		descfile.close();
		cout << "Motion description was written to file: " << filename << endl;

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
			for (int pnt = 0; pnt < lib::spkm::NUM_OF_MOTION_SEGMENTS + 1; ++pnt) {
				axis_pvt << (int) p(pnt, i) << ";" << (int) v(pnt, i) << ";" << (int) t(pnt) << ";\r\n";
			} //: for points
			// Close file for given axis.
			axis_pvt.close();
			cout << "PVT for axis " << i << " were written to file: " << filename << endl;

			// List of trajectory parameters for every segment.
			// Generate unique name.
			filename = dir + "axis" + boost::lexical_cast <std::string>(i) + "_m0123.csv";
			ofstream axis_m0123;
			axis_m0123.open(filename.c_str());
			// Write header.
			axis_m0123 << "m0w;m1w;m2w;m3w;\r\n";
			// Write parameters.
			for (int sgt = 0; sgt < lib::spkm::NUM_OF_MOTION_SEGMENTS; ++sgt) {
				axis_m0123 << motor_0w(sgt, i) << ";" << motor_1w(sgt, i) << ";" << motor_2w(sgt, i) << ";"
				<< motor_3w(sgt, i) << ";\r\n";
			} //: for segments
			// Close file for given axis.
			axis_m0123.close();
			cout << "Trajectory parameters for axis " << i << " were written to file: " << filename << endl;

			// Write
		} //: for axes
	} //: else
#endif

	// Check which motor moves.
	Eigen::Matrix <bool, 1, lib::spkm::NUM_OF_SERVOS> change;
	check_pvt_translocation <lib::spkm::NUM_OF_MOTION_SEGMENTS + 1, lib::spkm::NUM_OF_SERVOS> (p, change);

	// Execute motion
	if (!robot_test_mode) {
		// Setup motion parameters
		for (size_t i = 0; i < axes.size(); ++i) {

			// If no translocation is required for given axis - skip the motion (in order to save time).
			if (!change(i))
				continue;

			// Set motion parameters.
			axes[i]->setOperationMode(maxon::epos::OMD_INTERPOLATED_POSITION_MODE);
			axes[i]->setProfileVelocity(MotorVmax[i]);
			axes[i]->setProfileAcceleration(MotorAmax[i]);
			axes[i]->setProfileDeceleration(MotorAmax[i]);
			// TODO: setup acceleration and velocity limit values
			axes[i]->clearPvtBuffer();
			for (int pnt = 0; pnt < lib::spkm::NUM_OF_MOTION_SEGMENTS + 1; ++pnt) {
				axes[i]->setInterpolationDataRecord((int32_t) p(pnt, i), (int32_t) v(pnt, i), (uint8_t) t(pnt));
				printf("\rsend: %2d/%zu, free: %2d", pnt, i, axes[i]->getActualBufferSize());
				fflush(stdout);
			}
			printf("\n");

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
#if 1
		// Display axes movement.
		for (size_t i = 0; i < axes.size(); ++i) {
			cout << "Axis " << i << ": qc;rpm;ms;\r\n";
			for (int pnt = 0; pnt < lib::spkm::NUM_OF_MOTION_SEGMENTS + 1; ++pnt) {
				cout << (int) p(pnt, i) << ";" << (int) v(pnt, i) << ";" << (int) t(pnt) << ";\r\n";
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

void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{
	try {
		// Check controller state.
		check_controller_state();

		// we do not check the arm position when only lib::SET is set
		if (instruction.instruction_type != lib::SET) {
			switch (ecp_edp_cbuffer.get_pose_specification)
			{
				case lib::spkm::MOTOR:
#ifdef display_commands
					msg->message("EDP get_arm_position MOTOR");
#endif
					for (size_t i = 0; i < axes.size(); ++i) {
						if (robot_test_mode) {
							edp_ecp_rbuffer.epos_controller[i].position = current_motor_pos[i];
							edp_ecp_rbuffer.epos_controller[i].current = 0;
							edp_ecp_rbuffer.epos_controller[i].motion_in_progress = false;
						} else {
							current_motor_pos[i] = axes[i]->getActualPosition();
							edp_ecp_rbuffer.epos_controller[i].position = current_motor_pos[i];
							edp_ecp_rbuffer.epos_controller[i].current = axes[i]->getActualCurrent();
							edp_ecp_rbuffer.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
						}
					}
					break;
				case lib::spkm::JOINT:
#ifdef display_commands
					msg->message("EDP get_arm_position JOINT");
#endif
					// Read actual values from the hardware.
					if (!robot_test_mode) {
						for (size_t i = 0; i < axes.size(); ++i) {
							current_motor_pos[i] = axes[i]->getActualPosition();
							edp_ecp_rbuffer.epos_controller[i].current = axes[i]->getActualCurrent();
							edp_ecp_rbuffer.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
						}
					}

					// Do the calculation
					get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

					// Fill the values into a buffer
					for (int i = 0; i < number_of_servos; ++i) {
						edp_ecp_rbuffer.epos_controller[i].position = current_joints[i];
					}
					break;
				case lib::spkm::WRIST_ORIENTED_XYZ_EULER_ZYZ_WITH_TOOL:
					// In case of SYNC_TRAPEZOIDAL and NON_SYNC_TRAPEZOIDAL those two types of commands are executed in exactly the same way.
				case lib::spkm::XYZ_EULER_ZYZ:
#ifdef display_commands
					msg->message("EDP get_arm_position (WRIST) XYZ_EULER_ZYZ");
#endif
					// Return current end-effector pose if it is known (last motion was performed in the cartesian space).
					if (is_previous_cartesian_pose_known)
						edp_ecp_rbuffer.current_pose = current_end_effector_frame;
					else
						// Return identity.
						edp_ecp_rbuffer.current_pose.setIdentity();

					// Return additional informations regarding current and motion.
					if (!robot_test_mode) {
						for (size_t i = 0; i < axes.size(); ++i) {
							edp_ecp_rbuffer.epos_controller[i].current = axes[i]->getActualCurrent();
							edp_ecp_rbuffer.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
						}
					}
					break;
				case lib::spkm::TOOL_ORIENTED_XYZ_EULER_ZYZ_WITH_TOOL:
#ifdef display_commands
					msg->message("EDP get_arm_position TOOL_ORIENTED_XYZ_EULER_ZYZ_WITH_TOOL");
#endif
					// Return current end-effector pose if it is known (last motion was performed in the cartesian space).
					if (is_previous_cartesian_pose_known)
						edp_ecp_rbuffer.current_pose = current_shead_frame;
					else
						// Return identity.
						edp_ecp_rbuffer.current_pose.setIdentity();

					// Return additional informations regarding current and motion.
					if (!robot_test_mode) {
						for (size_t i = 0; i < axes.size(); ++i) {
							edp_ecp_rbuffer.epos_controller[i].current = axes[i]->getActualCurrent();
							edp_ecp_rbuffer.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
						}
					}

					break;
				default:
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

void effector::create_kinematic_models_for_given_robot(void)
{
	// Add main SPKM kinematics.
	add_kinematic_model(new kinematics::spkm::kinematic_model_spkm());
	// Set active model
	set_kinematic_model(0);
}

/*--------------------------------------------------------------------------*/
/*                           Utility routines                               */
/*--------------------------------------------------------------------------*/
void effector::create_threads()
{
	rb_obj = (boost::shared_ptr <common::reader_buffer>) new common::reader_buffer(*this);
	//vis_obj = (boost::shared_ptr <common::vis_server>) new common::vis_server(*this);
}

void effector::instruction_deserialization()
{
	memcpy(&ecp_edp_cbuffer, instruction.serialized_command, sizeof(ecp_edp_cbuffer));
}

void effector::reply_serialization(void)
{
	memcpy(reply.serialized_reply, &edp_ecp_rbuffer, sizeof(edp_ecp_rbuffer));
	assert(sizeof(reply.serialized_reply) >= sizeof(edp_ecp_rbuffer));
}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	manip_effector::single_thread_master_order(nm_task, nm_tryb);
}

} // namespace spkm
} // namespace edp
} // namespace mrrocpp
