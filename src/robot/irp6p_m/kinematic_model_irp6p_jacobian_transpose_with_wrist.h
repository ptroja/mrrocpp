/*!
 * @file
 * @brief File containing the IRp-6p with wrist (6DOFs) transposed jacobian based kinematic model class.
 *
 * @author Anna Maria Sibilska
 * @author tkornuta
 * @date 18.07.2007
 *
 * @ingroup KINEMATICS IRP6P_KINEMATICS irp6p_m
 */

#if !defined(_IRP6P_KIN_MODEL_WITH_WRIST_JACOBIAN_TRANSPOSE)
#define _IRP6P_KIN_MODEL_WITH_WRIST_JACOBIAN_TRANSPOSE

// Definicja klasy kinematic_model.
#include "robot/irp6p_m/kinematic_model_irp6p_with_wrist.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6p {

/*!
 *
 * @brief The IRp-6p with wrist (6DOFs) transposed jacobian based kinematic model class.
 *
 * @author Anna Maria Sibilska
 * @date 18.07.2007
 *
 * @ingroup KINEMATICS IRP6P_KINEMATICS
 */
class model_jacobian_transpose_with_wrist : public model_with_wrist
{

public:
	/**
	 * @brief Constructor.
	 * @param _number_of_servos Number of servos (joints).
	 */
	model_jacobian_transpose_with_wrist(int _number_of_servos);

	/**
	 * @brief Solves inverse kinematics utilizing the transposed jacobian. The new, 6th DOF is active.
	 *
	 * @param[out] local_desired_joints Computed join values (q0, q1, ...).
	 * @param[in] local_current_joints Current (in fact previous) internal values.
	 * @param[in] local_desired_end_effector_frame Given end-effector frame.
	 */
	virtual void
			inverse_kinematics_transform(lib::JointArray & local_desired_joints, lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame);

};//: kinematic_model_irp6p_jacobian_transpose_with_wrist

} // namespace irp6p
} // namespace kinematic
} // namespace mrrocpp

#endif
