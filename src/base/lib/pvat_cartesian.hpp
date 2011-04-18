/*!
 * @file pvat_cartesian.hpp
 * @brief Calculations related to the cartesian trajectory generation.
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
 * @brief Creates a table containing time slices.
 *
 * This method considers only constant (the same for every segment) time slices.
 *
 * @author tkornuta
 *
 * @param [out] motor_interpolationsVector containing interpolated poses.
 * @param [in] motion_time Total motion time.
 * @param [in] segment_time Time of motion for one segment (the same for every one).
 */
void pvat_divide_motion_time_into_constant_time_slices(
		std::vector < double > & time_slices,
		const double motion_time,
		const double segment_time
		)
{
	// Compute number of segments. Number of interpolated points is equal to n_segments + 1 (the starting point).
	unsigned int n_points =  motion_time/segment_time;

	// There must be some segments (besides we cannot divide by zero).
	assert (n_points!=0);

	for (int i = 0; i < n_points; ++i) {
		time_slices.push_back(segment_time);
	}
}


/**
 * @brief Computes interpolation motor positions for the cartesian trajectory generation.
 *
 * Method consideres also the different motion time for different segments!
 *
 * @author tkornuta
 *
 * @tparam N_AXES Number of manipulator axes.
 *
 * @param [out] motor_interpolationsVector containing interpolated poses.
 * @param [in] motion_time Total motion time.
 * @param [in] time_slices Times of motion for one segment (may be different for each segment!).
 * @param [in] model Kinematic model required for inverse kinematics computations.
 * @param [in] desired_joints_old Desired joint values that were required by previously received SET command (threated as current position of joints).
 * @param [in] current_end_effector_frame Homogeneous matrix containing current end effector pose.
 * @param [in] desired_end_effector_frame Homogeneous matrix containing desired end effector pose.
 */
template <unsigned int N_AXES>
void pvat_interpolate_motor_poses(
		std::vector < Eigen::Matrix <double, N_AXES, 1> > & motor_interpolations,
		const double motion_time,
		const std::vector < double > time_slices,
		mrrocpp::kinematics::common::kinematic_model* model,
		const lib::JointArray desired_joints_old,
		const mrrocpp::lib::Homog_matrix& current_end_effector_frame,
		const mrrocpp::lib::Homog_matrix& desired_end_effector_frame
		)
{
	// Manipulator has got to have some axes.
	assert (N_AXES!=0);
	// Check model.
	assert (model);
	// There must be some segments (besides we cannot divide by zero).
	assert (time_slices.size() != 0);
	// Number of interpolated points is equal to number of segments (thus time slices) + 1 (the starting point).

//	std::cout<<"Operational space: " << model->get_kinematic_model_label() << "\n";

	// Variable containing computed transformation from current end-effector post to the desired one.
	lib::Homog_matrix desired_relative_end_effector_frame;

	// Compute transformation from current to desired pose.
	desired_relative_end_effector_frame = !current_end_effector_frame * desired_end_effector_frame;

	std::cout << desired_relative_end_effector_frame << std::endl;

	// Extract translation and rotation (second one in the form of angle, axis and gamma).
	Xyz_Angle_Axis_Gamma_vector relative_xyz_aa_gamma;
	desired_relative_end_effector_frame.get_xyz_angle_axis_gamma(relative_xyz_aa_gamma);

	// Compute current P, v and gamma.
	Xyz_Angle_Axis_Gamma_vector current_xyz_aa_gamma;
	current_end_effector_frame.get_xyz_angle_axis_gamma(current_xyz_aa_gamma);

	// Delta variables.
	Xyz_Angle_Axis_Gamma_vector delta_xyz_aa_gamma;
	lib::Homog_matrix delta_ee_frame;
	// Interpolation variables.
	lib::Homog_matrix int_ee_frame;
	lib::JointArray int_joints(6);
	lib::MotorArray int_motors(6);
	// Set last joint settings.
	lib::JointArray int_joints_old = desired_joints_old;

	// Add current position as first one, thus there will be n+1 interpolation points.
	// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
	model->inverse_kinematics_transform(int_joints, desired_joints_old, current_end_effector_frame);
	// Transform joints to motors (and check motors/joints values).
	model->i2mp_transform(int_motors, int_joints);
	// Add motors to vector - first interpolation point.
	motor_interpolations.push_back(int_motors);

	// Temporary variables containing motion time from start to current position (sum of time slices).
	double last_summed = time_slices[0];
	double total_time_factor = last_summed/motion_time;

	// Compute interpolation points in motor positions.
	for (int i = 0; i < time_slices.size(); ++i) {
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
		int_ee_frame = current_end_effector_frame * delta_ee_frame;

//		std::cout << int_ee_frame << std::endl;

		// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
		model->inverse_kinematics_transform(int_joints, int_joints_old, int_ee_frame);

//		std::cout << int_joints << std::endl;

		// Transform joints to motors (and check motors/joints values).
		model->i2mp_transform(int_motors, int_joints);

//		std::cout << int_motors << std::endl;

		// Add motors to vector.
		motor_interpolations.push_back(int_motors);

		// Set last joint settings.
		int_joints_old = int_joints;
		// Add time slice related to this segment.
		if (i < time_slices.size()-1)
		{
			last_summed += time_slices[i+1];
			total_time_factor = last_summed/motion_time;
		}
	}

	// Display all motor poses.
	for(unsigned int l = 0; l < motor_interpolations.size(); ++l)
	{
		std::cout<<"Motor interpolation point no "<<l<<": "<<motor_interpolations[l].transpose()<<std::endl;
	}
}

template <unsigned int N_AXES>
void pvat_compute_tau_coefficients_matrix(
		)
{

}


/**
 * Compute parameters of the cartesian trajectory for the Maxon EPOS Interpolated Position Mode.
 * @tparam N number of axes
 * @param Vmax velocity limit vector for all axes.
 * @param Amax acceleration limit vector for all axes.
 * @param Dmax deacceleration limit vector for all axes.
 * @returns natenczas nie wiem co...
 */
template <unsigned int N>
double compute_pavt_cartesian_trajectory(
		const Matrix<double,N,1> & Vmax,
		const Matrix<double,N,1> & Amax,
		const Matrix<double,N,1> & Dmax
		)
{
	return 0.0;
}


} // namespace lib
} // namespace mrrocpp

#endif /* PAVT__CARTESIAN_HPP_ */

