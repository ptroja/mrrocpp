/*!
 * @file
 * @brief File containing the declaration of kinematic_model_with_local_corrector class.
 *
 * @author tkornuta
 * @date Nov 26, 2009
 *
 * @ingroup KINEMATICS
 */

#ifndef KINEMATIC_MODEL_WITH_LOCAL_CORRECTOR_H_
#define KINEMATIC_MODEL_WITH_LOCAL_CORRECTOR_H_

#include "base/kinematics/kinematic_model_with_tool.h"

namespace mrrocpp {
namespace kinematics {
namespace common {

/*!
 *
 * @brief Abstract class with methods for local end-effector position correction with the use of local correction matrices.
 *
 * @author tkornuta
 * @date Nov 26, 2009
 *
 * @ingroup KINEMATICS
 */
class kinematic_model_with_local_corrector : public mrrocpp::kinematics::common::kinematic_model_with_tool
{
protected:
	//! Calibration factor.
	double h;
	//! Correction vector.
	double V[6];
	//! Local correction matrix.
	double U[6][6];
	//! Local correction inverted matrix.
	double inv_U[6][6];

public:
	//! Destructor - empty.
	virtual ~kinematic_model_with_local_corrector()
	{
	}

	/**
	 * @brief Transformation related to the local correction matrix.
	 * @param current_end_effector_matrix Homogeneous matrix.
	 */
	virtual void local_corrector_transform(lib::Homog_matrix& current_end_effector_matrix);

	/**
	 * @brief Inverse transformation related to the local correction matrix.
	 * @param desired_end_effector_matrix Homogeneous matrix.
	 */
	virtual void local_corrector_inverse_transform(lib::Homog_matrix& desired_end_effector_matrix);

	/**
	 * @brief Computes external coordinates on the base of internal coordinates (i2e - internal to external).
	 * Utilizes the methods related to global reference frameattached tool, and local correctors computations.
	 * @param[in] local_current_joints Current joints values.
	 * @param[out] local_current_end_effector_frame Homogeneous matrix with computed end effector frame.
	 */
	virtual void
			i2e_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame);

	/**
	 * @brief Computes external coordinates on the base of internal coordinates, without the computations related with the attached tool.
	 * @param[in] local_current_joints Current joints values.
	 * @param[out] local_current_end_effector_frame Homogeneous matrix with computed end effector frame.
	 */
	virtual void
			i2e_wo_tool_transform(const lib::JointArray& local_current_joints, lib::Homog_matrix& local_current_end_effector_frame);

	/**
	 * @brief Computes internal coordinates basing on external coordinates (e2i - external to internal).
	 * Utilizes the methods related to global reference frameattached tool, and local correctors computations.
	 * @param[out] local_desired_joints Computed join values.
	 * @param[in] local_current_joints Current (in fact previous) internal values.
	 * @param[in] local_desired_end_effector_frame Given end-effector frame.
	 */
	virtual void
			e2i_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame);

	/**
	 * @brief Computes internal coordinates basing on external coordinates, without the computations related with the attached tool.
	 * @param[out] local_desired_joints Computed join values.
	 * @param[in] local_current_joints Current (in fact previous) internal values.
	 * @param[in] local_desired_end_effector_frame Given end-effector frame.
	 */
	virtual void
			e2i_wo_tool_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame);

};

} // namespace common
} // namespace kinematic
} // namespace mrrocpp

#endif /* KINEMATIC_MODEL_WITH_LOCAL_CORRECTOR_H_ */
