#include <iostream>
#include <cstdio>

#include <boost/foreach.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "robot/spkm/edp_e_spkm.h"
#include "base/edp/reader.h"
#include "base/edp/vis_server.h"
// Kinematyki.
#include "robot/spkm/kinematic_model_spkm.h"
#include "base/edp/manip_trans_t.h"
#include "robot/epos/epos_gen.h"

#include "robot/epos/epos.h"
#include "robot/epos/epos_access_usb.h"
#include "base/lib/pvt.hpp"

namespace mrrocpp {
namespace edp {
namespace spkm {

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	manip_effector::single_thread_master_order(nm_task, nm_tryb);
}

void effector::get_controller_state(lib::c_buffer &instruction)
{
	// False is the initial value
	controller_state_edp_buf.is_synchronised = false;
	controller_state_edp_buf.is_power_on = false;
	controller_state_edp_buf.is_robot_blocked = false;

	if (!robot_test_mode) {
		// Try to get state of each axis
		unsigned int referenced = 0;
		unsigned int powerOn = 0;
		unsigned int notInFaultState = 0;
		BOOST_FOREACH(epos::epos * node, axes)
					{
						try {
							// Check if in the FAULT state
							if (node->checkEPOSstate() == 11) {
								// Read number of errors
								int errNum = node->readNumberOfErrors();
								for (int i = 1; i <= errNum; ++i) {
									// Get the detailed error
									uint32_t errCode = node->readErrorHistory(i);

									msg->message(epos::epos::ErrorCodeMessage(errCode));
								}
							} else {
								notInFaultState++;
							}
							if (node->isReferenced()) {
								// Do not break from this loop so this is a also a preliminary axis error check
								referenced++;
							}
							powerOn++;
						} catch (...) {
							// Probably the axis is not powered on, do nothing.
						}
					}
		// Robot is synchronised if all axes are referenced
		controller_state_edp_buf.is_synchronised = (referenced == axes.size());
		controller_state_edp_buf.is_power_on = (powerOn == axes.size());
		controller_state_edp_buf.is_robot_blocked = (notInFaultState == axes.size());
	}

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
}

// Konstruktor.
effector::effector(lib::configurator &_config) :
	manip_effector(_config, lib::spkm::ROBOT_NAME)
{
	number_of_servos = lib::spkm::NUM_OF_SERVOS;

	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();

	if (!robot_test_mode) {
		// Create gateway object
		gateway = (boost::shared_ptr <epos::epos_base>) new epos::epos_access_usb();

		// Connect to the gateway
		gateway->open();

		// Create epos objects according to CAN ID-mapping
		axisA = (boost::shared_ptr <epos::epos>) new epos::epos(*gateway, 5);
		axisB = (boost::shared_ptr <epos::epos>) new epos::epos(*gateway, 4);
		axisC = (boost::shared_ptr <epos::epos>) new epos::epos(*gateway, 6);
		//		axis4 = (boost::shared_ptr<epos::epos>) new epos::epos(*gateway, 1);
		//		axis5 = (boost::shared_ptr<epos::epos>) new epos::epos(*gateway, 2);
		//		axis6 = (boost::shared_ptr<epos::epos>) new epos::epos(*gateway, 3);

		axes[0] = &(*axisA);
		axes[1] = &(*axisB);
		axes[2] = &(*axisC);
		//		axes[3] = &(*axis4);
		//		axes[4] = &(*axis5);
		//		axes[5] = &(*axis6);
	}
}

/*--------------------------------------------------------------------------*/
void effector::move_arm(const lib::c_buffer &instruction)
{
	try {
		switch (ecp_edp_cbuffer.variant)
		{
			case lib::spkm::CBUFFER_EPOS_MOTOR_COMMAND:
				msg->message("move_arm CBUFFER_EPOS_MOTOR_COMMAND");

				// Copy data directly from buffer to local data
				for (int i = 0; i < number_of_servos; ++i) {
					desired_motor_pos_new[i] = ecp_edp_cbuffer.epos_simple_command_structure.desired_position[i];
					std::cout << "MOTOR[ " << i << "]: "
							<< ecp_edp_cbuffer.epos_simple_command_structure.desired_position[i] << std::endl;
				}

				// Check desired joint values - instead of checking motors we need to check joints, because it is more robust. ;)
				if (is_synchronised()) {
					get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, desired_joints);
				}

				// Execute command
				for (std::size_t i = 0; i < axes.size(); ++i) {
					if (is_synchronised()) {
						std::cout << "MOTOR: moveAbsolute[" << i << "] ( " << desired_motor_pos_new[i] << ")"
								<< std::endl;
						if (!robot_test_mode) {
							axes[i]->moveAbsolute(desired_motor_pos_new[i]);
						} else {
							current_joints[i] = desired_motor_pos_new[i];
							current_motor_pos[i] = desired_motor_pos_new[i];
						}
					} else {
						std::cout << "MOTOR: moveRelative[" << i << "] ( " << desired_motor_pos_new[i] << ")"
								<< std::endl;
						if (!robot_test_mode) {
							axes[i]->moveRelative(desired_motor_pos_new[i]);
						} else {
							current_motor_pos[i] += desired_motor_pos_new[i];
						}
					}
				}
				break;
			case lib::spkm::CBUFFER_EPOS_JOINT_COMMAND:
				msg->message("move_arm CBUFFER_EPOS_JOINT_COMMAND");

				//			std::cout << "CBUFFER_EPOS_JOINT_COMMAND: desired_position[2]: "
				//					<< ecp_edp_cbuffer.epos_simple_command_structure.desired_position[2] << std::endl;

				// Read variables from communication buffer.
				for (int i = 0; i < number_of_servos; ++i) {
					desired_joints[i] = ecp_edp_cbuffer.epos_simple_command_structure.desired_position[i];
				}

				// Execute command
				if (is_synchronised()) {
					// Transform from joint to motors (and check motors/joints values).
					get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new, desired_joints);
					for (std::size_t i = 0; i < axes.size(); ++i) {
						std::cout << "JOINT: moveAbsolute[" << i << "] ( " << desired_motor_pos_new[i] << ")"
								<< std::endl;
						// Perform move.
						if (!robot_test_mode) {
							axes[i]->moveAbsolute(desired_motor_pos_new[i]);
						} else {
							current_joints[i] = desired_motor_pos_new[i];
							current_motor_pos[i] = desired_motor_pos_new[i];
						}
					}
				} else {
					for (std::size_t i = 0; i < axes.size(); ++i) {
						std::cout << "JOINT: moveRelative[" << i << "] ( " << desired_motor_pos_new[i] << ")"
								<< std::endl;
						// Perform move.
						if (!robot_test_mode) {
							axes[i]->moveRelative(desired_motor_pos_new[i]);
						} else {
							current_joints[i] += desired_motor_pos_new[i];
							current_motor_pos[i] += desired_motor_pos_new[i];
						}
					}
				}
				break;
			case lib::spkm::CBUFFER_EPOS_EXTERNAL_COMMAND:
				msg->message("move_arm CBUFFER_EPOS_EXTERNAL_COMMAND");
				{
					lib::Homog_matrix tmp_frame(ecp_edp_cbuffer.desired_frame);
					std::cout << tmp_frame << std::endl;
				}
				break;
			case lib::spkm::CBUFFER_EPOS_CUBIC_COMMAND: {
				lib::epos::epos_cubic_command epos_cubic_command_structure;
				memcpy(&epos_cubic_command_structure, &(ecp_edp_cbuffer.epos_cubic_command_structure), sizeof(epos_cubic_command_structure));
			}
				break;
			case lib::spkm::CBUFFER_EPOS_TRAPEZOIDAL_COMMAND: {

				if (!is_synchronised()) {
					std::cerr << "Robot is not synchronized for TRAPEZOIDAL profile motion" << std::endl;
					return;
				}

				//ecp_edp_cbuffer.epos_trapezoidal_command_structure
				Matrix <double, 3, 1> Delta, Vmax, Amax, Vnew, Anew, Dnew;

				for (int i = 0; i < 3; ++i) {
					Delta[i] = 0;
					Vmax[i] = 5000;
					Amax[i] = 2000;
				}

				// Calculate time of trapezoidal profile motion according to commanded acceleration and velocity limits
				//			double t =
				ppm <3> (Delta.cwise().abs(), Vmax, Amax, Vnew, Anew, Dnew);

				// Setup motion parameters
				for (std::size_t i = 0; i < axes.size(); ++i) {
					axes[i]->setOpMode(epos::epos::OMD_PROFILE_POSITION_MODE);
					axes[i]->writePositionProfileType(0); // Trapezoidal velocity profile
					axes[i]->writePositionProfileVelocity(Vnew[i]);
					axes[i]->writePositionProfileAcceleration(Anew[i]);
					axes[i]->writePositionProfileDeceleration(Dnew[i]);
					axes[i]->writeTargetPosition(Delta[i]);
				}

				// Start motion
				for (std::size_t i = 0; i < axes.size(); ++i) {
					// switch to absolute positioning, cancel possible ongoing operation first!
					axes[i]->writeControlword(0x3f);
				}
			}
				break;
			case lib::spkm::CBUFFER_EPOS_OPERATIONAL_COMMAND: {
				lib::epos::epos_operational_command epos_operational_command_structure;
				memcpy(&epos_operational_command_structure, &(ecp_edp_cbuffer.epos_operational_command_structure), sizeof(epos_operational_command_structure));

			}
				break;
			case lib::spkm::CBUFFER_EPOS_BRAKE_COMMAND: {
				if (!robot_test_mode) {
					// Execute command
					for (std::size_t i = 0; i < axes.size(); ++i) {
						// Brake with Quickstop command
						axes[i]->changeEPOSstate(epos::epos::QUICKSTOP);
					}
				}
			}
				break;
			default:
				break;
		}//: switch
	}//: try
	catch (mrrocpp::lib::exception::mrrocpp_non_fatal_error & e) {
		std::cerr << diagnostic_information(e);
	}//: catch non fatal error

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{
	// we do not check the arm position when only lib::SET is set
	if (instruction.instruction_type != lib::SET) {

		msg->message("EDP get_arm_position");
		switch (instruction.get_arm_type)
		{
			case lib::MOTOR:
				msg->message("EDP get_arm_position MOTOR");
				for (std::size_t i = 0; i < axes.size(); ++i) {
					if (robot_test_mode) {
						edp_ecp_rbuffer.epos_controller[i].position = current_motor_pos[i];
						edp_ecp_rbuffer.epos_controller[i].current = 0;
						edp_ecp_rbuffer.epos_controller[i].motion_in_progress = false;
						edp_ecp_rbuffer.epos_controller[i].buffer_full = false;
					} else {
						edp_ecp_rbuffer.epos_controller[i].position = axes[i]->readActualPosition();
						edp_ecp_rbuffer.epos_controller[i].current = axes[i]->readActualCurrent();
						edp_ecp_rbuffer.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
						//edp_ecp_rbuffer.epos_controller[i].buffer_full = ...
					}
				}
				break;
			case lib::JOINT:
				msg->message("EDP get_arm_position JOINT");

				// Read actual values from hardware
				if (robot_test_mode) {
					// Value of the current_motor_pos already filled in the move_arm() method.
				} else {
					for (std::size_t i = 0; i < axes.size(); ++i) {
						current_motor_pos[i] = axes[i]->readActualPosition();
					}
				}

				// Do the calculation
				get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

				// Fill the values into a buffer
				for (int i = 0; i < number_of_servos; ++i) {
					edp_ecp_rbuffer. epos_controller[i].position = current_joints[i];
				}
				break;
			case lib::FRAME: {
				msg->message("EDP get_arm_position FRAME");

				lib::Homog_matrix tmp_frame;

				tmp_frame.get_frame_tab(edp_ecp_rbuffer.current_frame);

			}
				break;
			default:
				break;

		}
	}

	reply.servo_step = step_counter;
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

void effector::synchronise(void)
{
	if (robot_test_mode) {
		controller_state_edp_buf.is_synchronised = true;
		return;
	}

	// reset controller
	BOOST_FOREACH(epos::epos * node, axes)
				{
					node->reset();
				}

	// switch to homing mode
	BOOST_FOREACH(epos::epos * node, axes)
				{
					node->setOpMode(epos::epos::OMD_HOMING_MODE);
				}

	// Do homing
	BOOST_FOREACH(epos::epos * node, axes)
				{
					node->startHoming();
				}

	// Loop until homing is finished
	bool finished;
	do {
		finished = true;
		BOOST_FOREACH(epos::epos * node, axes)
					{
						if (!node->isHomingFinished()) {
							finished = false;
						}
					}
	} while (!finished);

	// Hardcoded safety values
	// TODO: move to configuration file
	axisA->writeMinimalPositionLimit(-195000);
	axisA->writeMaximalPositionLimit(11500);
	axisB->writeMinimalPositionLimit(-282500);
	axisB->writeMaximalPositionLimit(11500);
	axisC->writeMinimalPositionLimit(-175000);
	axisC->writeMaximalPositionLimit(11000);

	// Just for testing
	//	axisA->writeMinimalPositionLimit(-100000);
	//	axisB->writeMinimalPositionLimit(-100000);
	//	axisC->writeMinimalPositionLimit(-100000);

	controller_state_edp_buf.is_synchronised = true;
}

/*--------------------------------------------------------------------------*/
void effector::create_threads()
{
	rb_obj = (boost::shared_ptr <common::reader_buffer>) new common::reader_buffer(*this);
	vis_obj = (boost::shared_ptr <common::vis_server>) new common::vis_server(*this);
}

void effector::instruction_deserialization()
{
	memcpy(&ecp_edp_cbuffer, instruction.arm.serialized_command, sizeof(ecp_edp_cbuffer));

	std::cerr << "EDP: " << ecp_edp_cbuffer << std::endl;
}

void effector::reply_serialization(void)
{
	memcpy(reply.arm.serialized_reply, &edp_ecp_rbuffer, sizeof(edp_ecp_rbuffer));
	assert(sizeof(reply.arm.serialized_reply) >= sizeof(edp_ecp_rbuffer));
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

