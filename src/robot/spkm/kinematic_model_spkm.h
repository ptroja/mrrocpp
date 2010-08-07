/*!
 * \file kinematic_model_spkm.h
 * \brief File containing declaration of the kinematic_model_spkm class.
 *
 * \author tkornuta
 * \date Jan 05, 2010
 */

#ifndef KINEMATIC_MODEL_SPKM_H_
#define KINEMATIC_MODEL_SPKM_H_

#include "base/kinematics/kinematic_model.h"
#include "robot/spkm/kinematic_parameters_spkm.h"
#include "base/kinematics/kinematic_model_with_tool.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm {

/*!
 * \class kinematic_model_spkm
 * \brief Class solving the problem of inverse kinematics for Exechon parallel kinematc machine with spherical wrist attached to its upper platform.
 *
 * \author tkornuta
 * \date Jan 05, 2010
 */
class kinematic_model_spkm: public common::kinematic_model
{
	protected:
		//! Kinematic parameters of both: parallel kinematic machine and spherical wrist attached to it.
		kinematic_parameters_spkm params;

		//! Sets parameters used by given kinematics model.
		void set_kinematic_parameters(void) { }

		//! Checks whether given motor increments are valid.
		void check_motor_position(const lib::MotorArray & motor_position);

		//! Checks whether given internal coordinates are valid.
		void check_joints(const lib::JointArray & q);

	public:
		//! Constructor.
		kinematic_model_spkm(void);

		//! Computes joint values basing on the motor increments - direct method not resolved, thus the method is empty.
		void mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints) { }

		//! Solves direct kinematics - not resolved, thus this method is empty.
		void direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame) { }

		//! Computes motor increments from joint values (i2mp from internal to motor position).
		void i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, lib::JointArray & local_desired_joints);

		//! Solves inverse kinematics.
		void inverse_kinematics_transform(lib::JointArray & local_desired_joints, lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame);

		/*! Computes platform pose on the base of given _O_S_P.
		 * \param _O_S_P Position of the middle of the spherical wrist (S) in the O (lower PM platform) reference frame.
		 * \return Platform pose in the form of e = <s_alpha,c_alpha,s_beta,c_beta, h>.
		 */
		Vector5d PKM_S_to_e(const Vector3d & _O_S_P);

		/*! Computes values of PM joints basing on given e.
		 * \param _e Platform pose in the form of e = <s_alpha,c_alpha,s_beta,c_beta, h>.
		 * \return Joints in the form of vector <qA,qB,qC>.
		 */
		Vector3d PKM_inverse_from_e(const Vector5d & _e);

		/*! Computes matrix O_P_T, representing the position  and orientation of upper platform (P) in relation to the lower one (O).
		 * \param _e Platform pose in the form of e = <s_alpha,c_alpha,s_beta,c_beta, h>.
		 * \return O_P_T.
		 */
		Homog4d PKM_O_P_T_from_e(const Vector5d & _e);

		/*! Computes values of SW joints basing on P_W_T.
		 \param _P_W_T Position of end of spherical wrist (W) with relation to its base, which is in fact upper platform reference frame (P).
		 \return Joints in the form of vector <q1,q2,q3>.
		 */
		Vector3d SW_inverse(const Homog4d & _P_W_T);
};

} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp


#endif

