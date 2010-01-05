// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota - klasa abstrakcyjna.
//				- definicja czesci metod klasy
//
// Autor:		tkornuta
// Data:		17.03.2007
// ------------------------------------------------------------------------

#include "kinematics/common/kinematic_model_with_tool.h"

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

void kinematic_model_with_tool::attached_tool_transform(lib::Homog_matrix& current_end_effector_matrix)
{
    current_end_effector_matrix *= tool;
}


void kinematic_model_with_tool::attached_tool_inverse_transform(lib::Homog_matrix& desired_end_effector_matrix)
{
    desired_end_effector_matrix *= (!tool);
}


void kinematic_model_with_tool::global_frame_transform(lib::Homog_matrix& current_end_effector_matrix)
{
    current_end_effector_matrix = (global_base * current_end_effector_matrix);
}


void kinematic_model_with_tool::global_frame_inverse_transform(lib::Homog_matrix& desired_end_effector_matrix)
{
    desired_end_effector_matrix = ((!global_base) * desired_end_effector_matrix);
}


void kinematic_model_with_tool::i2e_transform(const double* local_current_joints, lib::frame_tab* local_current_end_effector_frame)
{
	// Solution of the direct kinematics.
	direct_kinematics_transform(local_current_joints, local_current_end_effector_frame);

	// Create a copy of the end effector frame.
	lib::Homog_matrix local_current_end_effector_matrix(*local_current_end_effector_frame);

	// Computations related to the global reference frame.
	if (global_frame_computations)
		global_frame_transform(local_current_end_effector_matrix);

	// Computations related to the attached tool.
	if (attached_tool_computations)
		attached_tool_transform(local_current_end_effector_matrix);

	// Retrieve computations result.
	local_current_end_effector_matrix.get_frame_tab(*local_current_end_effector_frame);
}

void kinematic_model_with_tool::i2e_wo_tool_transform(const double* local_current_joints, lib::frame_tab* local_current_end_effector_frame)
{

	// Solution of the direct kinematics.
	direct_kinematics_transform(local_current_joints, local_current_end_effector_frame);

	// Create a copy of the end effector frame.
	lib::Homog_matrix local_current_end_effector_matrix(*local_current_end_effector_frame);

	// Computations related to the global reference frame.
	if (global_frame_computations)
		global_frame_transform(local_current_end_effector_matrix);

	// Retrieve computations result.
	local_current_end_effector_matrix.get_frame_tab(*local_current_end_effector_frame);

}

void kinematic_model_with_tool::e2i_transform(double* local_desired_joints, double* local_current_joints, lib::frame_tab* local_desired_end_effector_frame)
{
	// Copy end effector frame.
	lib::Homog_matrix local_desired_end_effector_matrix(*local_desired_end_effector_frame);

	// Compute inverse transformation related to the attached tool.
	if (attached_tool_computations)
		attached_tool_inverse_transform(local_desired_end_effector_matrix);

	// Compute transformation related to the global frame - if required.
	if (global_frame_computations)
		global_frame_inverse_transform(local_desired_end_effector_matrix);

	// Retrieve computations result.
	local_desired_end_effector_matrix.get_frame_tab(*local_desired_end_effector_frame);

	// Compute inverse kinematics transformation.
	inverse_kinematics_transform(local_desired_joints, local_current_joints, local_desired_end_effector_frame);

}

void kinematic_model_with_tool::e2i_wo_tool_transform(double* local_desired_joints, double* local_current_joints, lib::frame_tab* local_desired_end_effector_frame)
{
	// Copy end effector frame.
	lib::Homog_matrix local_desired_end_effector_matrix(*local_desired_end_effector_frame);

	// Compute transformation related to the global frame - if required.
	if (global_frame_computations)
		global_frame_inverse_transform(local_desired_end_effector_matrix);

	// Retrieve computations result.
	local_desired_end_effector_matrix.get_frame_tab(*local_desired_end_effector_frame);

	// Compute inverse kinematics transformation.
	inverse_kinematics_transform(local_desired_joints, local_current_joints, local_desired_end_effector_frame);
}



} // namespace common
} // namespace kinematics
} // namespace mrrocpp

