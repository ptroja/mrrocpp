/*!
 * @file pvat_cartesian.hpp
 * @brief Calculations related to the cartesian trajectory generation.
 *
 * Most of the equations are implemented due to the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system" report.
 *
 * @author tkornuta
 *
 * @ingroup LIB
 */
#ifndef PAVT__CARTESIAN_HPP_
#define PAVT__CARTESIAN_HPP_

#include <Eigen/Core>
#include <Eigen/Array>
#include <cmath>
#include <iostream>
#include <cassert>
#include <vector>
#include <boost/foreach.hpp>

#include "base/kinematics/kinematic_model.h"

namespace mrrocpp {
namespace lib {


// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN


/**
 * @brief Creates a vector containing time slices.
 *
 * This method considers only constant (the same for every segment) time slices.
 *
 * @author tkornuta
 *
 * @tparam N_SEGMENTS Number of motion segments.
 *
 * @param [out] time_deltas_ Computed time slices (one for each segment).
 * @param [in] motion_time_ Total motion time.
 */
template <unsigned int N_SEGMENTS>
void pvat_divide_motion_time_into_constant_time_deltas(
		Eigen::Matrix <double, N_SEGMENTS, 1> & time_deltas_,
		const double motion_time_
		)
{
	// There must be some segments (besides we cannot divide by zero).
	assert (N_SEGMENTS!=0);
	double segment_time = motion_time_ / N_SEGMENTS;

	for (int i = 0; i < N_SEGMENTS; ++i) {
		time_deltas_(i) = segment_time;
	}

	for (int i = 0; i < N_SEGMENTS; ++i) {
		std::cout<<" "<<time_deltas_(i);
	}
	std::cout<<std::endl;
}



/**
 * @brief Computes interpolation motor positions for the cartesian trajectory generation.
 *
 * Method consideres also the different motion time for different segments!
 * Number of interpolated points is equal to number of segments (thus time slices) + 1 (the starting point).
 *
 * @author tkornuta
 *
 * @tparam N_POINTS Number of interpolation points.
 * @tparam N_AXES Number of manipulator axes.
 *
 * @param [out] motor_interpolations_ Matrix containing interpolated motor poses.
 * @param [in] motion_time_ Total motion time.
 * @param [in] time_deltas_ Times of motion for one segment (may be different for each segment!).
 * @param [in] model_ Kinematic model required for inverse kinematics computations.
 * @param [in] desired_joints_old_ Desired joint values that were required by previously received SET command (threated as current position of joints).
 * @param [in] current_end_effector_frame_ Homogeneous matrix containing current end effector pose.
 * @param [in] desired_end_effector_frame_ Homogeneous matrix containing desired end effector pose.
 */
template <unsigned int N_POINTS, unsigned int N_AXES>
void pvat_interpolate_motor_poses(
		Eigen::Matrix <double, N_POINTS, N_AXES> & motor_interpolations_,
		const double motion_time_,
		const Eigen::Matrix <double, N_POINTS-1, 1> time_deltas_,
		mrrocpp::kinematics::common::kinematic_model* model_,
		const lib::JointArray desired_joints_old_,
		const mrrocpp::lib::Homog_matrix& current_end_effector_frame_,
		const mrrocpp::lib::Homog_matrix& desired_end_effector_frame_
		)
{
	// Manipulator has got to have some axes.
	assert (N_AXES>0);
	// There must be some segments (besides we cannot divide by zero).
	assert (N_POINTS>1);
	// Check model.
	assert (model_);

//	std::cout<<"Operational space: " << model->get_kinematic_model_label() << "\n";

	// Variable containing computed transformation from current end-effector post to the desired one.
	lib::Homog_matrix desired_relative_end_effector_frame;

	// Compute transformation from current to desired pose.
	desired_relative_end_effector_frame = !current_end_effector_frame_ * desired_end_effector_frame_;

	std::cout << desired_relative_end_effector_frame << std::endl;

	// Extract translation and rotation (second one in the form of angle, axis and gamma).
	Xyz_Angle_Axis_Gamma_vector relative_xyz_aa_gamma;
	desired_relative_end_effector_frame.get_xyz_angle_axis_gamma(relative_xyz_aa_gamma);

	// Compute current P, v and gamma.
	Xyz_Angle_Axis_Gamma_vector current_xyz_aa_gamma;
	current_end_effector_frame_.get_xyz_angle_axis_gamma(current_xyz_aa_gamma);

	// Delta variables.
	Xyz_Angle_Axis_Gamma_vector delta_xyz_aa_gamma;
	lib::Homog_matrix delta_ee_frame;
	// Interpolation variables.
	lib::Homog_matrix int_ee_frame;
	lib::JointArray int_joints(N_AXES);
	lib::MotorArray int_motors(N_AXES);
	// Set last joint settings.
	lib::JointArray int_joints_old = desired_joints_old_;

	// Add current position as first one, thus there will be n+1 interpolation points.
	// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
	model_->inverse_kinematics_transform(int_joints, desired_joints_old_, current_end_effector_frame_);
	// Transform joints to motors (and check motors/joints values).
	model_->i2mp_transform(int_motors, int_joints);
	// Add motors to vector - first interpolation point.
	motor_interpolations_.row(0) = int_motors.transpose();

	// Temporary variables containing motion time from start to current position (sum of time slices).
	double last_summed = time_deltas_(0);
	double total_time_factor = last_summed/motion_time_;

	// Compute interpolation points in motor positions.
	for (int i = 0; i < N_POINTS-1; ++i) {
		// Compute delta in the angle axis gamma representation.
		delta_xyz_aa_gamma
			// Px, Py, Pz.
			<< relative_xyz_aa_gamma(0) * total_time_factor, relative_xyz_aa_gamma(1) * total_time_factor, relative_xyz_aa_gamma(2) * total_time_factor,
			// vx, vy, vz (constant).
			relative_xyz_aa_gamma(3), relative_xyz_aa_gamma(4), relative_xyz_aa_gamma(5),
			// Gamma.
			relative_xyz_aa_gamma(6) * total_time_factor;

		// Compute delta frame.
		delta_ee_frame.set_from_xyz_angle_axis_gamma(delta_xyz_aa_gamma);

		// Compute desired interpolation end effector frame.
		int_ee_frame = current_end_effector_frame_ * delta_ee_frame;

//		std::cout << int_ee_frame << std::endl;

		// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
		model_->inverse_kinematics_transform(int_joints, int_joints_old, int_ee_frame);

//		std::cout << int_joints << std::endl;

		// Transform joints to motors (and check motors/joints values).
		model_->i2mp_transform(int_motors, int_joints);

//		std::cout << int_motors << std::endl;

		// Add motors to vector.
		motor_interpolations_.row(i+1) = int_motors.transpose();

		// Set last joint settings.
		int_joints_old = int_joints;
		// Add time slice related to this segment.
		if (i < N_POINTS-2)
		{
			last_summed += time_deltas_(i+1);
			total_time_factor = last_summed/motion_time_;
		}
	}

	// Display all motor interpolation poses.
	for(unsigned int l = 0; l < N_POINTS; ++l)
	{
		std::cout<<"Motor interpolation point no "<<l<<": "<<motor_interpolations_.row(l)<<std::endl;
	}

}

/**
 * @brief Compute motor_deltas for segments.
 *
 * Matrix in the form (equation 1.48) from the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system".
 *
 * @tparam N_SEGMENTS number of motion segments.
 * @tparam N_AXES Number of manipulator axes.
 *
 * @param [out] motor_deltas_for_segments_ Motor position increments to be realized in given segment.
 * @param [in] motor_interpolations_ Matrix containing interpolated motor poses.
 */
template <unsigned int N_SEGMENTS, unsigned int N_AXES>
void pvat_compute_motor_deltas_for_segments(
		Eigen::Matrix <double, N_SEGMENTS, N_AXES> & motor_deltas_for_segments_,
		const Eigen::Matrix <double, N_SEGMENTS+1, N_AXES> motor_interpolations_
		)
{
	for (int segment = 0; segment < N_SEGMENTS; ++segment) {
		for (int axis = 0; axis < N_AXES; ++axis) {
			motor_deltas_for_segments_(segment, axis) = motor_interpolations_(segment+1, axis) - motor_interpolations_(segment, axis);
		}
	}

	// Display all motor increments.
	for(unsigned int l = 0; l < N_SEGMENTS; ++l)
	{
		std::cout<<"Motor increments for segment "<<l<<": "<<motor_deltas_for_segments_.row(l)<<std::endl;
	}
}


/**
 * @brief Computes matrix of the tau coefficients. The matrix is identical for every axis, thus is independent of axes number.
 *
 * Matrix in the form (equation 1.48) from the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system".
 *
 * @tparam N_SEGMENTS number of motion segments.
 *
 * @param [out] tau_coefficients_ Returned matrix.
 * @param [in] time_deltas_ Times of motion for one segment (may be different for each segment!).
 */
template <unsigned int N_SEGMENTS>
void pvat_compute_tau_coefficients_matrix(
		Eigen::Matrix <double, N_SEGMENTS, N_SEGMENTS> & tau_coefficients_,
		Eigen::Matrix <double, N_SEGMENTS, 1> & time_deltas_
		)
{
	// Zero matrix.
	tau_coefficients_ = Eigen::Matrix <double, N_SEGMENTS, N_SEGMENTS>::Zero();

	// First row.
	tau_coefficients_(0, 0) = time_deltas_(0) * 2.0/3.0;
	tau_coefficients_(0, 1) = time_deltas_(0) / 3.0;
	// Rows 2..n-2.
	for (int i = 1; i < N_SEGMENTS-1; ++i) {
		tau_coefficients_(i, i-1) = time_deltas_(i-1) / 3.0;
		tau_coefficients_(i, i) = (time_deltas_(i) + time_deltas_(i-1)) * 2.0/3.0;
		tau_coefficients_(i, i+1) = time_deltas_(i) / 3.0;
	}
	// Last row.
	tau_coefficients_(N_SEGMENTS-1, N_SEGMENTS-2) = time_deltas_(N_SEGMENTS-2) / 3.0;
	tau_coefficients_(N_SEGMENTS-1, N_SEGMENTS-1) = time_deltas_(N_SEGMENTS-2) * 2.0/3.0 + time_deltas_(N_SEGMENTS-1) / 2.0;

	std::cout<<"tau_coefficients:\n"<<tau_coefficients_<<std::endl;
}

/**
 * @brief Computes matrix of the right side coefficients for every motor.
 *
 * Matrix in the form (equation 1.48) from the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system".
 *
 * @author tkornuta
 *
 * @tparam N_SEGMENTS number of motion segments.
 * @tparam N_AXES Number of manipulator axes.
 *
 * @param [out] right_side_coefficients_ Returned matrix.
 * @param [in] time_deltas_ Times of motion for one segment (may be different for each segment!).
 */
template <unsigned int N_SEGMENTS, unsigned int N_AXES>
void pvat_compute_right_side_coefficients_vector(
		Eigen::Matrix <double, N_SEGMENTS, N_AXES> & right_side_coefficients_,
		const Eigen::Matrix <double, N_SEGMENTS, N_AXES> motor_deltas_,
		const Eigen::Matrix <double, N_SEGMENTS, 1> time_deltas_
		)
{
	for (int motor = 0; motor < N_AXES; ++motor) {
		// First segment.
		right_side_coefficients_(0, motor) = motor_deltas_(0, motor) / time_deltas_(0);
		// 1..n-1 segments.
		for (int segment = 1; segment < N_SEGMENTS-1; ++segment) {
			right_side_coefficients_(segment, motor) = (motor_deltas_(segment, motor))/ (time_deltas_(segment)) - (motor_deltas_(segment-1, motor))/ (time_deltas_(segment-1));
		}

		// Last segment.
		right_side_coefficients_(N_SEGMENTS-1, motor) = (motor_deltas_(N_SEGMENTS-1, motor) * 3.0)/ (time_deltas_(N_SEGMENTS-1) * 2.0) - (motor_deltas_(N_SEGMENTS-2, motor))/ (time_deltas_(N_SEGMENTS-2));
	}

	std::cout<<"right_side_coefficients:\n"<<right_side_coefficients_<<std::endl;
}



} // namespace lib
} // namespace mrrocpp

#endif /* PAVT__CARTESIAN_HPP_ */

