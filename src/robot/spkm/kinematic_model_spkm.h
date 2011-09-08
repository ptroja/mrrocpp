/*!
 * @file
 * @brief File containing declaration of the kinematic_model_spkm class.
 *
 * @author Tomasz Kornuta
 * @date Jan 05, 2010
 *
 * @ingroup KINEMATICS SIF_KINEMATICS spkm
 */

#ifndef KINEMATIC_MODEL_SPKM_H_
#define KINEMATIC_MODEL_SPKM_H_

#include "base/kinematics/kinematic_model.h"
#include "const_spkm.h"
#include "kinematic_parameters_spkm.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm {

/*!
 *
 * @brief Class solving the problem of inverse kinematics for Exechon parallel kinematic machine (PM) with spherical wrist (SW) attached to its upper platform.
 *
 * @author Tomasz Kornuta
 * @date Jan 05, 2010
 *
 * @ingroup KINEMATICS SIF_KINEMATICS
 */
class kinematic_model_spkm : public common::kinematic_model
{
protected:
	//! Kinematic parameters of both: parallel kinematics machine (PM) and spherical wrist (SW) attached to it.
	kinematic_parameters_spkm params;

	//! Upper platform pose - computed during the IK and used for Cartesian limits verification.
	Homog4d O_P_T;

	//! Sets parameters used by given kinematics model - empty.
	void set_kinematic_parameters(void)
	{
	}

public:
	//! Constructor.
	kinematic_model_spkm(void);

	/*!
	 * @brief Checks whether given motor increments are valid.
	 * @param motor_position Motor position to be validated.
	 */
	void check_motor_position(const lib::MotorArray & motor_position) const;

	/*!
	 * @brief Checks whether given internal coordinates are valid.
	 * @param q Joints to be validated.
	 */
	void check_joints(const lib::JointArray & q) const;

	/**
	 * @brief Checks whether given Cartesian pose is valid - but in this case computations are based on the upper PM platform pose computed in the IK.
	 * @param h_ Cartesian pose to be validated (in this case not used!).
	 */
	void check_cartesian_pose(const lib::Homog_matrix& H_) const;

	/*!
	 * @brief Computes internal coordinates for given the motor increments (position) values.
	 *
	 * @param[in] local_current_motor_pos Motor increments.
	 * @param[out] local_current_joints Computed joints.
	 */
	void mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints);

	/*!
	 * @brief Computes motor increments from internal coordinates.
	 *
	 * @param[out] local_desired_motor_pos_new Computed motor increment.
	 * @param[in] local_desired_joints Current joints settings.
	 */
	void i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints);

	/*!
	 * @brief Solves direct kinematics - not implemented.
	 *
	 * @param[in] local_current_joints Given internal (joints) values.
	 * @param[out] local_current_end_effector_frame Computed end-effector frame (a homogeneous matrix).
	 */
	void direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame)
	{
		//TODO: Throw an adequate exception.
	}

	/*!
	 * @brief Solves inverse kinematics for the whole PKM.
	 *
	 * @param[out] local_desired_joints Computed join values.
	 * @param[in] local_current_joints Current (in fact previously desired) joint values.
	 * @param[in] local_desired_end_effector_frame Given end-effector frame.
	 */
	void inverse_kinematics_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame);

	/*!
	 * @brief Computes platform pose on the base of given _O_S_T.
	 *
	 * Implementation according to Matteo Zoppi and Dimiter Zlatanov formulas from "Position and velocity kinematics
	 * of the Exechon parallel machine" report, reduced by the unnecessary parameters (deltas, l12A, l12C, hA, hC).
	 *
	 * @param O_S_T_ Pose of the middle of the spherical wrist (S) in the O (lower PM platform) reference frame.
	 * @return Platform pose in the form of e = <s_alpha,c_alpha,s_beta,c_beta, h>.
	 */
	Vector5d PM_S_to_e(const Homog4d & O_S_T_);

	/*!
	 * @brief Computes values of PM joints basing on given e.
	 *
	 * Implementation according to Matteo Zoppi and Dimiter Zlatanov formulas from "Position and velocity kinematics
	 * of the Exechon parallel machine" report, reduced by the unnecessary parameters (deltas, l12A, l12C, hA, hC).
	 *
	 * @param e_ Platform pose in the form of e = <s_alpha,c_alpha,s_beta,c_beta, h>.
	 * @return Joints in the form of vector <qA,qB,qC>.
	 */
	Vector3d PM_inverse_from_e(const Vector5d & e_);

	/*!
	 * @brief Computes matrix O_P_T, representing the position  and orientation of upper platform (P) in relation to the lower one (O).
	 *
	 * @param e_ Platform pose in the form of e = <s_alpha,c_alpha,s_beta,c_beta, h>.
	 * @return O_P_T.
	 */
	Homog4d PM_O_P_T_from_e(const Vector5d & e_);

	/*!
	 * @brief Computes SW inverse kinematics transform - values of SW thetas basing on desired twist of spherical wrist.
	 *
	 * @param [in] P_W_T_ Position of end of spherical wrist (W) with relation to its base, which is in fact upper platform reference frame (P) (the twist of the wrist).
	 * @param[in] local_current_joints Current (in fact previously desired) joint values.
	 * @return Joints in the form of vector <q1,q2,q3>.
	 */
	Vector3d SW_inverse(const Homog4d & P_W_T_, const lib::JointArray & local_current_joints);

	// You must overload "operator new" so that it generates 16-bytes-aligned pointers
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp


#endif

