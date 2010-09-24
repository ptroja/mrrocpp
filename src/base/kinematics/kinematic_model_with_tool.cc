/*!
 * @file
 * @brief File containing the definition of kinematic_model_with_tool class methods.
 *
 * @author tkornuta
 * @date Nov 26, 2009
 *
 * @ingroup KINEMATICS
 */

#include "base/kinematics/kinematic_model_with_tool.h"

namespace mrrocpp {
namespace kinematics {
namespace common {

kinematic_model_with_tool::kinematic_model_with_tool(void)
{
    // Don't compute global frame transformation - as default.
    global_frame_computations = false;
    // Don't compute tool transformation - as default.
    attached_tool_computations = false;
}

kinematic_model_with_tool::~kinematic_model_with_tool(void)
{
}

void kinematic_model_with_tool::attached_tool_transform(lib::Homog_matrix& homog_matrix)
{
	homog_matrix *= tool;
}


void kinematic_model_with_tool::attached_tool_inverse_transform(lib::Homog_matrix& homog_matrix)
{
	homog_matrix *= (!tool);
}


void kinematic_model_with_tool::global_frame_transform(lib::Homog_matrix& homog_matrix)
{
	homog_matrix = (global_base * homog_matrix);
}


void kinematic_model_with_tool::global_frame_inverse_transform(lib::Homog_matrix& homog_matrix)
{
	homog_matrix = ((!global_base) * homog_matrix);
}


void kinematic_model_with_tool::i2e_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame)
{
	// Solution of the direct kinematics.
	direct_kinematics_transform(local_current_joints, local_current_end_effector_frame);

	// Create a copy of the end effector frame.
	lib::Homog_matrix local_current_end_effector_matrix(local_current_end_effector_frame);

	// Computations related to the global reference frame.
	if (global_frame_computations)
		global_frame_transform(local_current_end_effector_matrix);

	// Computations related to the attached tool.
	if (attached_tool_computations)
		attached_tool_transform(local_current_end_effector_matrix);

	// Retrieve computations result.
	local_current_end_effector_frame = local_current_end_effector_matrix;
}

void kinematic_model_with_tool::i2e_wo_tool_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame)
{
	// Solution of the direct kinematics.
	direct_kinematics_transform(local_current_joints, local_current_end_effector_frame);

	// Create a copy of the end effector frame.
	lib::Homog_matrix local_current_end_effector_matrix(local_current_end_effector_frame);

	// Computations related to the global reference frame.
	if (global_frame_computations)
		global_frame_transform(local_current_end_effector_matrix);

	// Retrieve computations result.
	local_current_end_effector_frame = local_current_end_effector_matrix;
}

void kinematic_model_with_tool::e2i_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{
	// Copy end effector frame.
	lib::Homog_matrix local_desired_end_effector_matrix(local_desired_end_effector_frame);

	// Compute inverse transformation related to the attached tool.
	if (attached_tool_computations)
		attached_tool_inverse_transform(local_desired_end_effector_matrix);

	// Compute transformation related to the global frame - if required.
	if (global_frame_computations)
		global_frame_inverse_transform(local_desired_end_effector_matrix);

	// Compute inverse kinematics transformation.
	inverse_kinematics_transform(local_desired_joints, local_current_joints, local_desired_end_effector_matrix);
}

void kinematic_model_with_tool::e2i_wo_tool_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{
	// Copy end effector frame.
	lib::Homog_matrix local_desired_end_effector_matrix(local_desired_end_effector_frame);

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

