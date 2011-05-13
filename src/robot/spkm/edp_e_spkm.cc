#include <iostream>
#include <fstream>
#include <cstdio>

#include <boost/foreach.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

#include "robot/spkm/edp_e_spkm.h"
#include "base/edp/reader.h"
//#include "base/edp/vis_server.h"

#include "robot/spkm/kinematic_model_spkm.h"
#include "robot/spkm/kinematic_parameters_spkm.h"
#include "base/edp/manip_trans_t.h"

#include "robot/epos/epos.h"
#include "robot/epos/epos_access_usb.h"
#include "base/lib/pvt.hpp"
#include "base/lib/pvat_cartesian.hpp"

#include "robot/spkm/exceptions.h"

namespace mrrocpp {
namespace edp {
namespace spkm {

using namespace mrrocpp::lib;
using namespace std;

const uint32_t effector::Vdefault[6] = { 5000UL, 5000UL, 5000UL, 5000UL, 5000UL, 5000UL };
const uint32_t effector::Adefault[6] = { 2000UL, 2000UL, 2000UL, 2000UL, 2000UL, 2000UL };
const uint32_t effector::Ddefault[6] = { 2000UL, 2000UL, 2000UL, 2000UL, 2000UL, 2000UL };

effector::effector(common::shell &_shell) :
	manip_effector(_shell, lib::spkm::ROBOT_NAME)
{
	// Set number of servos.
	number_of_servos = lib::spkm::NUM_OF_SERVOS;

	// Create all kinematic models for SPKM.
	create_kinematic_models_for_given_robot();

	// Zero motors and set initial joint values.
	reset_variables();

	if (!robot_test_mode) {
		// Create gateway object.
		gateway = (boost::shared_ptr <epos::epos_access>) new epos::epos_access_usb();

		// Connect to the gateway.
		gateway->open();

		// Create epos objects according to CAN ID-mapping.
		axisA = (boost::shared_ptr <epos::epos>) new epos::epos(*gateway, 5);
		axisB = (boost::shared_ptr <epos::epos>) new epos::epos(*gateway, 4);
		axisC = (boost::shared_ptr <epos::epos>) new epos::epos(*gateway, 6);
		axis1 = (boost::shared_ptr <epos::epos>) new epos::epos(*gateway, 3);
		axis2 = (boost::shared_ptr <epos::epos>) new epos::epos(*gateway, 2);
		axis3 = (boost::shared_ptr <epos::epos>) new epos::epos(*gateway, 1);

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
	//current_end_effector_frame.setIdentity(4,4);
	//desired...
	is_previous_cartesian_pose_known = false;
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
		for (size_t i = 0; i < axes.size(); ++i) {
			try {
				// Check if in the FAULT state
				if (axes[i]->checkEPOSstate() == 11) {
					// Read number of errors
					int errNum = axes[i]->readNumberOfErrors();
					for (int j = 1; j <= errNum; ++j) {
						// Get the detailed error
						uint32_t errCode = axes[i]->readErrorHistory(j);

						msg->message(mrrocpp::lib::FATAL_ERROR, string("axis ") + axesNames[i] + ": " + epos::epos::ErrorCodeMessage(errCode));
					}
				} else {
					notInFaultState++;
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

void effector::synchronise(void)
{
	if (robot_test_mode) {
		controller_state_edp_buf.is_synchronised = true;
		return;
	}

	// switch to homing mode
	BOOST_FOREACH(epos::epos * node, axes)
	{
		node->setOperationMode(epos::epos::OMD_HOMING_MODE);
	}

	// reset controller
	BOOST_FOREACH(epos::epos * node, axes)
	{
		node->reset();
	}

	// Do homing with preconfigured parameters
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

	// Hardcoded safety values.
	// TODO: move to configuration file?
	for (size_t i = 0; i < axes.size(); ++i) {
		axes[i]->writeMinimalPositionLimit(kinematics::spkm::kinematic_parameters_spkm::lower_motor_pos_limits[i] - 1);
		axes[i]->writeMaximalPositionLimit(kinematics::spkm::kinematic_parameters_spkm::upper_motor_pos_limits[i] + 1);
	}

	// Move the longest linear axis to the 'zero' position with a fast motion command
	axisB->writeProfileVelocity(5000UL);
	axisB->writeProfileAcceleration(1000UL);
	axisB->writeProfileDeceleration(1000UL);
	axisB->moveAbsolute(-57500);

	// Just for testing if limits actually work
	//	axisA->writeMinimalPositionLimit(-100000);
	//	axisB->writeMinimalPositionLimit(-100000);
	//	axisC->writeMinimalPositionLimit(-100000);

	// Reset internal state of the motor positions
	for (int i = 0; i < number_of_servos; ++i) {
		current_motor_pos[i] = desired_motor_pos_old[i] = 0;
	}

	// Compute joints positions in the home position
	get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

	// Now the robot is synchronised
	controller_state_edp_buf.is_synchronised = true;
}

void effector::move_arm(const lib::c_buffer &instruction)
{
	// Variable denoting that the move is performed in the cartesian space.
	//bool cartesian_space_move = true;

	try {
		switch (ecp_edp_cbuffer.variant)
		{
			case lib::spkm::POSE:
				try {
					switch (ecp_edp_cbuffer.pose_specification)
					{
						case lib::spkm::MOTOR:
							// Copy data directly from buffer
							for (int i = 0; i < number_of_servos; ++i) {
								desired_motor_pos_new[i] = ecp_edp_cbuffer.motor_pos[i];
								cout << "MOTOR[ " << i << "]: " << desired_motor_pos_new[i] << endl;
							}

							// Check desired joint values if they are absolute.
							if (is_synchronised()) {
								get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, desired_joints);
							}
							break;
						case lib::spkm::JOINT:
							// Copy data directly from buffer
							for (int i = 0; i < number_of_servos; ++i) {
								desired_joints[i] = ecp_edp_cbuffer.joint_pos[i];
								cout << "JOINT[ " << i << "]: " << desired_joints[i] << endl;
							}

							if (is_synchronised()) {
								// Transform desired joint to motors (and check motors/joints values).
								get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new, desired_joints);
							} else {
								// Throw non-fatal error - this mode requires synchronization.
								BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_robot_unsynchronized());
							}

							break;
						case lib::spkm::FRAME:
							// debug display
							cout << "FRAME: [";
							for (unsigned int i = 0; i < 6; ++i) {
								cout << ecp_edp_cbuffer.goal_pos[i] << ", ";
							}
							cout << "]\n";

							if (is_synchronised()) {
								// Retrieve the desired homogeneous matrix on the base of received six  variables - a Euler Z-Y-Z representation.
								desired_end_effector_frame.set_from_xyz_euler_zyz_without_limits(Xyz_Euler_Zyz_vector(ecp_edp_cbuffer.goal_pos));
								cout << desired_end_effector_frame << endl;

								// Check the motion type.
								if (ecp_edp_cbuffer.motion_variant == lib::epos::OPERATIONAL) {
									// In operational space the previous cartesian pose is required.
									if (!is_previous_cartesian_pose_known)
										BOOST_THROW_EXCEPTION(mrrocpp::edp::spkm::nfe_current_cartesian_pose_unknown());
									// Rest of computations is performed elsewhere...
								} else {
									// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
									get_current_kinematic_model()->inverse_kinematics_transform(desired_joints, desired_joints_old, desired_end_effector_frame);

									// Transform joints to motors (and check motors/joints values).
									get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new, desired_joints);

									// Remember the currently desired joints as old.
									desired_joints_old = desired_joints;
								}
							} else {
								// Throw non-fatal error - this mode requires synchronization.
								BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_robot_unsynchronized());
							}
							break;
						default:
							// Throw non-fatal error - invalid pose specification.
							BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_pose_specification());
							break;
					}//: switch (ecp_edp_cbuffer.pose_specification)
				} catch (boost::exception &e_) {
					// TODO add other context informations that are available.
					e_ << mrrocpp::edp::spkm::pose_specification(ecp_edp_cbuffer.pose_specification);
					// Throw the catched exception.
					throw;
				}

				// Note: at this point we assume, that desired_motor_pos_new holds a validated data.

				switch (ecp_edp_cbuffer.motion_variant)
				{
					case lib::epos::NON_SYNC_TRAPEZOIDAL:
						// Execute command
						for (size_t i = 0; i < axes.size(); ++i) {
							if (is_synchronised()) {
								cout << "MOTOR: moveAbsolute[" << i << "] ( " << desired_motor_pos_new[i] << ")"
										<< endl;
								if (!robot_test_mode) {
									axes[i]->writeProfileVelocity(Vdefault[i]);
									axes[i]->writeProfileAcceleration(Adefault[i]);
									axes[i]->writeProfileDeceleration(Ddefault[i]);
									axes[i]->moveAbsolute(desired_motor_pos_new[i]);
								} else {
									current_joints[i] = desired_joints[i];
									current_motor_pos[i] = desired_motor_pos_new[i];
								}
							} else {
								cout << "MOTOR: moveRelative[" << i << "] ( " << desired_motor_pos_new[i] << ")"
										<< endl;
								if (!robot_test_mode) {
									axes[i]->writeProfileVelocity(Vdefault[i]);
									axes[i]->writeProfileAcceleration(Adefault[i]);
									axes[i]->writeProfileDeceleration(Ddefault[i]);
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
							cout << "new - old[" << i << "]: " << desired_motor_pos_new[i] << " - "
									<< desired_motor_pos_old[i] << " = " << Delta[i] << endl;
							Vmax[i] = Vdefault[i] / epos::epos::SECONDS_PER_MINUTE;
							Amax[i] = Adefault[i];
						}

						// Calculate time of trapezoidal profile motion according to commanded acceleration and velocity limits
						double t = ppm <6> (Delta, Vmax, Amax, Vnew, Anew, Dnew);

						cerr << "Delta:\n" << Delta << endl << "Vmax:\n" << Vmax << endl << "Amax:\n"
								<< Amax << endl << endl;

						if (t > 0) {
							// debug display
							cerr << "Vnew:\n" << Vnew << endl << "Anew:\n" << Anew << endl << "Dnew:\n"
									<< Dnew << endl << endl;

							if (!robot_test_mode) {
								// Setup motion parameters
								for (size_t i = 0; i < axes.size(); ++i) {
									if (Delta[i] != 0) {
										axes[i]->setOperationMode(epos::epos::OMD_PROFILE_POSITION_MODE);
										axes[i]->writePositionProfileType(0); // Trapezoidal velocity profile
										axes[i]->writeProfileVelocity(Vnew[i] * epos::epos::SECONDS_PER_MINUTE);
										axes[i]->writeProfileAcceleration(Anew[i]);
										axes[i]->writeProfileDeceleration(Dnew[i]);
										axes[i]->writeTargetPosition(desired_motor_pos_new[i]);
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
					case lib::epos::OPERATIONAL: {
						// Check whether current cartesian pose (in fact the one where the previous motion ended) is known.
						// This ins fact is done earlier, in the lib::spkm::POSE == FRAME section, but I'm leaving this here for the clearance of solution.
						if (!is_previous_cartesian_pose_known)
							BOOST_THROW_EXCEPTION(mrrocpp::edp::spkm::nfe_current_cartesian_pose_unknown());

						// Position, Velocity, Acceleration, Deacceleration - for all axes.
						//Matrix <double, 6, 1> P, V, A, D;

						// Calculate time - currently the motion time is set to 5s.
						// TODO: analyze required (desired) movement time -> III cases: t<t_req, t=t_req, t>t_req.
						double motion_time = 2;

						// Constant time for one segment - 250ms.
						//double segment_time = 1;//0.25;

						// Divide motion time into segments (time slices).
						Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, 1> time_deltas;
						pvat_divide_motion_time_into_constant_time_deltas <lib::spkm::NUM_OF_MOTION_SEGMENTS> (time_deltas, motion_time);

						// Interpolate motor poses - equal to number of segments +1 (the start pose).
						Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS+1, lib::spkm::NUM_OF_SERVOS> motor_interpolations;
						pvat_cubic_polynomial_interpolate_motor_poses <lib::spkm::NUM_OF_MOTION_SEGMENTS+1, lib::spkm::NUM_OF_SERVOS> (motor_interpolations, motion_time, time_deltas, get_current_kinematic_model(), desired_joints_old, current_end_effector_frame, desired_end_effector_frame);
						//pvat_linear_interpolate_motor_poses <lib::spkm::NUM_OF_MOTION_SEGMENTS+1, lib::spkm::NUM_OF_SERVOS> (motor_interpolations, motion_time, time_deltas, get_current_kinematic_model(), desired_joints_old, current_end_effector_frame, desired_end_effector_frame);

						// Compute motor_deltas for segments.
						Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_deltas_for_segments;
						pvat_compute_motor_deltas_for_segments <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> (motor_deltas_for_segments, motor_interpolations);

						// Compute tau coefficient matrix of the (1.48) equation.
						Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_MOTION_SEGMENTS> tau_coefficients;
						pvat_compute_tau_coefficients_matrix <lib::spkm::NUM_OF_MOTION_SEGMENTS> (tau_coefficients, time_deltas);

						// Compute right side vector of the (1.48) equation - for all motors!!
						Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> right_side_coefficients;
						pvat_compute_right_side_coefficients_vector <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> (right_side_coefficients, motor_deltas_for_segments, time_deltas);

						// Compute 2w polynomial coefficients for all motors!!
						Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_2w;
						pvat_compute_motor_2w_polynomial_coefficients <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> (motor_2w, tau_coefficients, right_side_coefficients);

						// Compute 1w polynomial coefficients for all motors!!
						Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_1w;
						pvat_compute_motor_1w_polynomial_coefficients <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> (motor_1w, motor_2w, motor_deltas_for_segments, time_deltas);

						// Compute 3w polynomial coefficients for all motors!!
						Eigen::Matrix <double,lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_3w;
						pvat_compute_motor_3w_polynomial_coefficients <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> (motor_3w, motor_2w, motor_deltas_for_segments, time_deltas);

						// Compute 0w polynomial coefficients for all motors!!
						Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> motor_0w;
						pvat_compute_motor_0w_polynomial_coefficients <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> (motor_0w, motor_interpolations);

						cout << "time_deltas = [ \n" << time_deltas << "\n ]; \n";
						cout << "m0w = [\n" << motor_0w <<  "\n ]; \n";
						cout << "m1w = [\n" << motor_1w <<  "\n ]; \n";
						cout << "m2w = [\n" << motor_2w <<  "\n ]; \n";
						cout << "m3w = [\n" << motor_3w <<  "\n ]; \n";

						// Compute PVT triplets for generated segments.
						// Two additional points are added - first related to the start position and second it required by the EPOS (pos;0;0;).
						Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS+1, lib::spkm::NUM_OF_SERVOS> p;
						Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS+1, lib::spkm::NUM_OF_SERVOS> v;
						Eigen::Matrix <double, lib::spkm::NUM_OF_MOTION_SEGMENTS+1, 1> t;
						pvat_compute_pvt_triplets_for_epos <lib::spkm::NUM_OF_MOTION_SEGMENTS+1, lib::spkm::NUM_OF_SERVOS> (p, v, t, time_deltas, motor_3w, motor_2w, motor_1w, motor_0w);

/*						cout<<"p = [ \n"<<p << "\n ]; \n";
						cout<<"v = [ \n"<<v << "\n ]; \n";
						cout<<"t = [ \n"<<t << "\n ]; \n";*/

						// Recalculate units: p[qc], v[rpm (revolutions per minute) per second], t[miliseconds].
						for (int mtr = 0; mtr < lib::spkm::NUM_OF_SERVOS; ++mtr) {
							for (int pnt = 0; pnt < lib::spkm::NUM_OF_MOTION_SEGMENTS+1; ++pnt) {
								v(pnt, mtr) *= 60.0 / kinematics::spkm::kinematic_parameters_spkm::encoder_resolution[mtr];
							}
							//p.transpose().row(mtr) /= kinematics::spkm::kinematic_parameters_spkm::encoder_resolution[mtr];
/*							v.transpose().row(mtr) = v.transpose().row(mtr) * epos::epos::SECONDS_PER_MINUTE /
									kinematics::spkm::kinematic_parameters_spkm::encoder_resolution[mtr];*/
						}
						// Recalculate time to [ms].
						t *= 1000;

/*						cout<<" !Values after units recalculations!\n";
						cout<<"p = [ \n"<<p << "\n ]; \n";
						cout<<"v = [ \n"<<v << "\n ]; \n";
						cout<<"t = [ \n"<<t << "\n ]; \n";*/

						// Execute motion
						if (!robot_test_mode) {
							// Setup motion parameters
							for (size_t i = 0; i < axes.size(); ++i) {
								axes[i]->setOperationMode(epos::epos::OMD_INTERPOLATED_POSITION_MODE);
								// TODO: setup acceleration and velocity limit values
								axes[i]->clearPvtBuffer();
								for (int pnt = 0; pnt < lib::spkm::NUM_OF_MOTION_SEGMENTS+1; ++pnt) {
									axes[i]->writeInterpolationDataRecord((int32_t) p(pnt,i), (int32_t) v(pnt,i), (uint8_t) t(pnt));
									printf("\rsend: %2d/%d, free: %2d", pnt, i, axes[i]->readActualBufferSize());
									fflush(stdout);
								}
								printf("\n");

								const epos::UNSIGNED16 status = axes[i]->readInterpolationBufferStatus();

								if (axes[i]->checkInterpolationBufferWarning(status)) {
									axes[i]->printInterpolationBufferStatus(status);
								}

								if (axes[i]->checkInterpolationBufferError(status)) {
									// FIXME: this should be done in a separate exception, which does not benlong
									//        to the kinematics::spkm namespace.
									printf("InterpolationBufferStatus for axis %d: 0x%04X\n", i, status);
									BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_pose_specification());
								}
							}
						} else {
							// Display the 2nd axis movement.
							//ofstream myfile;
							//myfile.open ("/home/ptrojane/korni/pkm2.csv");

							cout<<"qc;rpm;ms;\r\n";
							for (int pnt = 0; pnt < lib::spkm::NUM_OF_MOTION_SEGMENTS+1; ++pnt) {
								cout<< (int)p(pnt,1) <<";"<< (int)v(pnt,1) << ";"<< (int)t(pnt) << ";\r\n";
							}
	//						myfile<< (int)p(lib::spkm::NUM_OF_MOTION_SEGMENTS,1) <<";0;0;";
							//myfile.close();
						}

						// Start motion
						for (size_t i = 0; i < axes.size(); ++i) {
							if (!robot_test_mode) {
								// FIXME: this motion type should be initiated with a CAN broadcast message
								axes[i]->startInterpolatedPositionMotion();
							} else {
								current_joints[i] = desired_joints[i];
								current_motor_pos[i] = desired_motor_pos_new[i];
							}
						}
					}
						break;
					default:
						// Throw non-fatal error - motion type not supported.
						BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_motion_type());
						break;
				}//: switch (ecp_edp_cbuffer.motion_variant)
				break;
			case lib::spkm::QUICKSTOP:
				if (!robot_test_mode) {
					// Execute command
					BOOST_FOREACH(epos::epos * node, axes)
								{
									// Brake with Quickstop command
									node->changeEPOSstate(epos::epos::QUICKSTOP);
								}
				}
				// Internal position counters need not be updated
				return;
			case lib::spkm::CLEAR_FAULT:
				BOOST_FOREACH(epos::epos * node, axes)
							{
								node->printEPOSstate();

								// Check if in a FAULT state
								if (node->checkEPOSstate() == 11) {
									epos::UNSIGNED8 errNum = node->readNumberOfErrors();
									cerr << "readNumberOfErrors() = " << (int) errNum << endl;
									for (epos::UNSIGNED8 i = 1; i <= errNum; ++i) {

										epos::UNSIGNED32 errCode = node->readErrorHistory(i);

										cerr << epos::epos::ErrorCodeMessage(errCode) << endl;
									}
									if (errNum > 0) {
										node->clearNumberOfErrors();
									}
									node->changeEPOSstate(epos::epos::FAULT_RESET);
								}

								// Change to the operational mode
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

		// Check whether the motion was performed in the cartesian space - then we know where manipulator will be when the next command arrives:).
		is_previous_cartesian_pose_known = (ecp_edp_cbuffer.pose_specification == lib::spkm::FRAME);
		if (is_previous_cartesian_pose_known)
			current_end_effector_frame = desired_end_effector_frame;
		/*		if (cartesian_space_move)
		 is_previous_cartesian_pose_known = true;
		 else
		 is_previous_cartesian_pose_known = false;*/
	} catch (mrrocpp::lib::exception::mrrocpp_non_fatal_error & e_) {
		is_previous_cartesian_pose_known = false;
		HANDLE_MRROCPP_ERROR(e_)
	}
}

void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{
	// we do not check the arm position when only lib::SET is set
	if (instruction.instruction_type != lib::SET) {
		switch (instruction.get_arm_type)
		{
			case lib::MOTOR:
				msg->message("EDP get_arm_position MOTOR");
				for (size_t i = 0; i < axes.size(); ++i) {
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
				if (!robot_test_mode) {
					for (size_t i = 0; i < axes.size(); ++i) {
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

				edp_ecp_rbuffer.current_frame = lib::Homog_matrix();
			}
				break;
			default:
				break;

		}
	}

	reply.servo_step = step_counter;
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
	memcpy(&ecp_edp_cbuffer, instruction.arm.serialized_command, sizeof(ecp_edp_cbuffer));
}

void effector::reply_serialization(void)
{
	memcpy(reply.arm.serialized_reply, &edp_ecp_rbuffer, sizeof(edp_ecp_rbuffer));
	assert(sizeof(reply.arm.serialized_reply) >= sizeof(edp_ecp_rbuffer));
}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	manip_effector::single_thread_master_order(nm_task, nm_tryb);
}

} // namespace spkm


namespace common {

// Create spkm effector.
effector* return_created_efector(common::shell &_shell)
{
	return new spkm::effector(_shell);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
