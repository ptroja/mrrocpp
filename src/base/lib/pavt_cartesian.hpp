/*!
 * @file pavt_cartesian.hpp
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

#include "base/kinematics/kinematic_model.h"

namespace mrrocpp {
namespace lib {




// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

/**
 * @briefComputes interpolation poses for the cartesian trajectory generation purposes.
 *
 * Currently only constant changes (in terms of are pose deltas) are taken into consideration.
 *
 * @author tkornuta
 *
 * @tparam N_AXES Number of manipulator axes.
 *
 * @param n_points Number of interpolation nodes.
 * @param model Kinematic model required for inverse kinematics computations.
 * @param desired_joints_old Desired joint values that were required by previously received SET command (threated as current position of joints).
 * @param current_end_effector_frame Homogeneous matrix containing current end effector pose.
 * @param desired_end_effector_frame Homogeneous matrix containing desired end effector pose.
 * @returns natenczas nie wiem co...
 */
template <unsigned int N_AXES>
double compute_cartesian_trajectory_interpolated_poses(
		unsigned int n_points,
		mrrocpp::kinematics::common::kinematic_model* model,
		const lib::JointArray desired_joints_old,
		const mrrocpp::lib::Homog_matrix& current_end_effector_frame,
		const mrrocpp::lib::Homog_matrix& desired_end_effector_frame
		)
{
	// There must be some segments (besides we cannot divide by zero).
	assert (n_points!=0);
	// Manipulator has got to have some axes.
	assert (N_AXES!=0);
	// Check model.
	assert (model);

	std::cout<<"Operational space: " << model->get_kinematic_model_label() << "\n";

	// Variable containing computed transformation from current end-effector post to the desired one.
	lib::Homog_matrix desired_relative_end_effector_frame;

	// Compute transformation from current to desired pose.
	desired_relative_end_effector_frame = !current_end_effector_frame * desired_end_effector_frame;

	std::cout << desired_relative_end_effector_frame << std::endl;

	// Compute relative P, v and gamma.
	Xyz_Angle_Axis_Gamma_vector relative_xyz_aa_gamma;
	desired_relative_end_effector_frame.get_xyz_angle_axis_gamma(relative_xyz_aa_gamma);
	// Compute P,v and gamma deltas - constant for each segment.
	double Px_delta = relative_xyz_aa_gamma(0)/n_points;
	double Py_delta = relative_xyz_aa_gamma(1)/n_points;
	double Pz_delta = relative_xyz_aa_gamma(2)/n_points;
	double gamma_delta = relative_xyz_aa_gamma(6)/n_points;

	// Compute current P, v and gamma.
	Xyz_Angle_Axis_Gamma_vector current_xyz_aa_gamma;
	current_end_effector_frame.get_xyz_angle_axis_gamma(current_xyz_aa_gamma);

	// Vector containing interpolated poses.
	std::vector < Eigen::Matrix <double, N_AXES, 1> > motor_interpolations;
	// Delta variables.
	Xyz_Angle_Axis_Gamma_vector delta_xyz_aa_gamma;
	lib::Homog_matrix delta_ee_frame;
	// Interpolation variables.
	lib::Homog_matrix int_ee_frame;
	lib::JointArray int_joints;
	lib::MotorArray int_motors;
	// Set last joint settings.
	lib::JointArray int_joints_old = desired_joints_old;

	// Compute interpolation points in motor positions.
	for (int i = 0; i < n_points; ++i) {
		// Compute delta in the angle axis gamma representation.
		delta_xyz_aa_gamma
			// Px, Py, Pz.
			<< Px_delta *(i+1), Py_delta *(i+1), Pz_delta *(i+1),
			// vx, vy, vz (constant).
			relative_xyz_aa_gamma(3), relative_xyz_aa_gamma(4), relative_xyz_aa_gamma(5),
			// Gamma.
			gamma_delta *(i+1);

		// Compute delta frame.
		delta_ee_frame.set_from_xyz_angle_axis_gamma(delta_xyz_aa_gamma);

		// Compute desired interpolation end effector frame.
		int_ee_frame = current_end_effector_frame * delta_ee_frame;

		std::cout << int_ee_frame << std::endl;

		// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
		model->inverse_kinematics_transform(int_joints, int_joints_old, int_ee_frame);

		std::cout << int_joints << std::endl;

		// Transform joints to motors (and check motors/joints values).
/*		model->i2mp_transform(int_motors, int_joints);

		std::cout << int_motors << std::endl;

		motor_interpolations.push_back(int_motors);

		// Set last joint settings.
		int_joints_old = int_joints;*/
	}

	return 0.0;
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

