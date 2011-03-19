/*!
 * @file
 * @brief File containing definitions of kinematic_model_with_local_corrector class methods.
 *
 * @author tkornuta
 * @date Nov 26, 2009
 *
 * @ingroup KINEMATICS
 */

#include "base/kinematics/kinematic_model_with_local_corrector.h"

namespace mrrocpp {
namespace kinematics {
namespace common {

void kinematic_model_with_local_corrector::local_corrector_transform(lib::Homog_matrix& current_end_effector_matrix)
{
	// std::cout<<" local_corrector_transform: before \n"<<current_end_effector_matrix<<std::endl;
	lib::Xyz_Euler_Zyz_vector d;
	current_end_effector_matrix.get_xyz_euler_zyz(d);
	// Transform the pose from XYZ_Euler_ZYZ to required form.
	double x[6];
	x[0] = d[0] * 1000;
	x[1] = d[1] * 1000;
	x[2] = d[2] * 1000;
	x[3] = d[3] * h;
	x[4] = d[4] * h;
	x[5] = d[5] * h;
	double z[6];
	// Compute corrected values.
	for (int i = 0; i < 6; i++) {
		z[i] = 0;
		for (int j = 0; j < 6; j++) {
			z[i] += U[i][j] * x[j];
		}//: for
		z[i] += V[i];
	}//: for
	// Transform corrected pose to the XYZ_Euler_ZYZ form.
	d[0] = z[0] / 1000;
	d[1] = z[1] / 1000;
	d[2] = z[2] / 1000;
	d[3] = z[3] / h;
	d[4] = z[4] / h;
	d[5] = z[5] / h;
	// Set computed corrected pose.
	current_end_effector_matrix.set_from_xyz_euler_zyz(d);
	// std::cout<<" local_corrector_transform: corrected \n"<<current_end_effector_matrix<<std::endl;
}

void kinematic_model_with_local_corrector::local_corrector_inverse_transform(lib::Homog_matrix& desired_end_effector_matrix)
{
	// std::cout<<" local_corrector_inverse_transform: before \n"<<desired_end_effector_matrix<<std::endl;
	lib::Xyz_Euler_Zyz_vector d;
	desired_end_effector_matrix.get_xyz_euler_zyz(d);
	// Transform corrected pose from the XYZ_Euler_ZYZ to required form.
	double z[6];
	z[0] = d[0] * 1000;
	z[1] = d[1] * 1000;
	z[2] = d[2] * 1000;
	z[3] = d[3] * h;
	z[4] = d[4] * h;
	z[5] = d[5] * h;
	double x[6];
	double zminV[6];
	// Compute original values.
	for (int i = 0; i < 6; i++)
		zminV[i] = z[i] - V[i];
	for (int i = 0; i < 6; i++) {
		x[i] = 0;
		for (int j = 0; j < 6; j++) {
			x[i] += inv_U[i][j] * zminV[j];
		}//: for
	}//: for
	// Transform pose to the the XYZ_Euler_ZYZ form.
	d[0] = x[0] / 1000;
	d[1] = x[1] / 1000;
	d[2] = x[2] / 1000;
	d[3] = x[3] / h;
	d[4] = x[4] / h;
	d[5] = x[5] / h;
	// Set computed, original pose.
	desired_end_effector_matrix.set_from_xyz_euler_zyz(d);
	// std::cout<<" local_corrector_inverse_transform: original \n"<<desired_end_effector_matrix<<std::endl;
}

void kinematic_model_with_local_corrector::i2e_transform(const lib::JointArray& local_current_joints, lib::Homog_matrix& local_current_end_effector_frame)
{
	// Solution of the direct kinematics.
	direct_kinematics_transform(local_current_joints, local_current_end_effector_frame);

	// Create a copy of the end effector frame.
	lib::Homog_matrix local_current_end_effector_matrix(local_current_end_effector_frame);

	// Computations related to the global reference frame.
	if (global_frame_computations)
		global_frame_transform(local_current_end_effector_matrix);

	// Computations related to the local correction matrix.
	local_corrector_transform(local_current_end_effector_matrix);

	// Computations related to the attached tool.
	if (attached_tool_computations)
		attached_tool_transform(local_current_end_effector_matrix);

	// Retrieve computations result.
	local_current_end_effector_frame = local_current_end_effector_matrix;
}

void kinematic_model_with_local_corrector::i2e_wo_tool_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame)
{

	// Solution of the direct kinematics.
	direct_kinematics_transform(local_current_joints, local_current_end_effector_frame);

	// Create a copy of the end effector frame.
	lib::Homog_matrix local_current_end_effector_matrix(local_current_end_effector_frame);

	// Computations related to the global reference frame.
	if (global_frame_computations)
		global_frame_transform(local_current_end_effector_matrix);

	// Computations related to the local correction matrix.
	local_corrector_transform(local_current_end_effector_matrix);

	// Retrieve computations result.
	local_current_end_effector_frame = local_current_end_effector_matrix;

}

void kinematic_model_with_local_corrector::e2i_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{
	// Copy end effector frame.
	lib::Homog_matrix local_desired_end_effector_matrix(local_desired_end_effector_frame);

	// Compute inverse transformation related to the attached tool.
	if (attached_tool_computations)
		attached_tool_inverse_transform(local_desired_end_effector_matrix);

	// Compute inverse local correction transformation.
	local_corrector_inverse_transform(local_desired_end_effector_matrix);

	// Compute transformation related to the global frame - if required.
	if (global_frame_computations)
		global_frame_inverse_transform(local_desired_end_effector_matrix);

	// Compute inverse kinematics transformation.
	inverse_kinematics_transform(local_desired_joints, local_current_joints, local_desired_end_effector_matrix);

}

void kinematic_model_with_local_corrector::e2i_wo_tool_transform(lib::JointArray& local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{
	// Copy end effector frame.
	lib::Homog_matrix local_desired_end_effector_matrix(local_desired_end_effector_frame);

	// Compute inverse local correction transformation.
	local_corrector_inverse_transform(local_desired_end_effector_matrix);

	// Compute transformation related to the global frame - if required.
	if (global_frame_computations)
		global_frame_inverse_transform(local_desired_end_effector_matrix);

	// Retrieve computations result.
	//local_desired_end_effector_frame = local_desired_end_effector_matrix;

	// Compute inverse kinematics transformation.
	inverse_kinematics_transform(local_desired_joints, local_current_joints, local_desired_end_effector_matrix);
}

} // namespace common
} // namespace kinematics
} // namespace mrrocpp
