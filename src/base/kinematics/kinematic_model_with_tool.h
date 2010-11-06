/*!
 * @file
 * @brief File containing the declaration of the kinematic_model_with_tool class.
 *
 * @author tkornuta
 * @date Jan 04, 2010
 *
 * @ingroup KINEMATICS
 */

#if !defined(__EDP_KIN_MODEL)
#define __EDP_KIN_MODEL

#include "base/lib/impconst.h"
#include "base/kinematics/kinematic_model.h"

namespace mrrocpp {
namespace kinematics {
namespace common {

/*!
 *
 * @brief Abstract class with methods for robots with tools attached to their end-effectors.
 * Class contains also methods for computations of robot base transformation to global reference frame.
 *
 * @author tkornuta
 * @date Jan 04, 2010
 *
 * @ingroup KINEMATICS
 */
class kinematic_model_with_tool : public mrrocpp::kinematics::common::kinematic_model
{
protected:
	/**
	 * @brief Flag related to the computations to the global reference frame.
	 * False by default - global frame transformation computations are turned off.
	 */
	bool global_frame_computations;

	/**
	 * @brief Flag related to the attached tool computations.
	 * False by default - tool transformation computations are turned off.
	 */
	bool attached_tool_computations;

public:

	//! Homogeneous matrix representing the transformation between the robot base and global reference frame.
	lib::Homog_matrix global_base;

	//! Homogeneous matrix representing the transformation between the end effector and tool attached to its end.
	lib::Homog_matrix tool;

	//! Class constructor - empty.
	kinematic_model_with_tool();

	//! Class virtual destructor - empty.
	virtual ~kinematic_model_with_tool();

	/**
	 * @brief Computes external coordinates on the base of internal coordinates (i2e - internal to external).
	 * Utilizes the methods related to global reference frame and attached tool computations.
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
			i2e_wo_tool_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame);

	/**
	 * @brief Computes internal coordinates basing on external coordinates (e2i - external to internal).
	 * Utilizes the methods related to global reference frame and attached tool computations.
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

	/**
	 * @brief Computes robot base transformation to global reference frame.
	 * @param[in,out] homog_matrix End-effector homogeneous matrix.
	 */
	void global_frame_transform(lib::Homog_matrix& homog_matrix);

	/**
	 * @brief Computes inverse base-global transformation - from global reference frame to robot base frame.
	 * @param[in,out] homog_matrix End-effector homogeneous matrix.
	 */
	void global_frame_inverse_transform(lib::Homog_matrix& homog_matrix);

	/**
	 * @brief Computes transformation of end-effector frame to attached tool frame.
	 * @param[in,out] homog_matrix End-effector homogeneous matrix.
	 */
	void attached_tool_transform(lib::Homog_matrix& homog_matrix);

	/**
	 * @brief Computes inverse end-effector-tool transformation.
	 * @param[in,out] homog_matrix End-effector homogeneous matrix.
	 */
	void attached_tool_inverse_transform(lib::Homog_matrix& homog_matrix);

};//: kinematic_model

} // namespace common
} // namespace kinematic
} // namespace mrrocpp

#endif
