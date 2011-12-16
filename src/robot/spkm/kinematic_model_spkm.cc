/*!
 * @file
 * @brief File containing definition of kinematic_model_spkm class methods.
 *
 * @author Tomasz Kornuta
 * @date Jan 05, 2010
 *
 * @ingroup KINEMATICS SIF_KINEMATICS spkm
 */

#include <cmath>
#include "base/lib/com_buf.h"
#include "exceptions.h"
#include "kinematic_model_spkm.h"

using namespace mrrocpp::edp::exception;
using namespace mrrocpp::edp::spkm;
using namespace std;

namespace mrrocpp {
namespace kinematics {
namespace spkm {

//! Prints reference frames.
#define DEBUG_KINEMATICS 0

//! Eps used in Spherical wrist inverse kinematics.
#define EPS 1.0e-10



kinematic_model_spkm::kinematic_model_spkm(void)
{
	// Set model name.
	set_kinematic_model_label("PKM 6DOF (SW+PM) kinematic model. PM inverse kinematics by D.Zlatanow and M.Zoppi");
}

void kinematic_model_spkm::check_motor_position(const lib::MotorArray & motor_position) const
{
	// Check upper limit for every motor.
	for (int i = 0; i < 6; ++i) {
		if (motor_position[i] > params.upper_motor_pos_limits[i])
			BOOST_THROW_EXCEPTION(nfe_motor_limit() << motor_number(i) << limit_type(UPPER_LIMIT) << desired_value(motor_position[i]));
		else if (motor_position[i] < params.lower_motor_pos_limits[i])
			BOOST_THROW_EXCEPTION(nfe_motor_limit() << motor_number(i) << limit_type(LOWER_LIMIT) << desired_value(motor_position[i]));
	}
}

void kinematic_model_spkm::check_joints(const lib::JointArray & q) const
{
	// Check joint limit for every axis.
	for (int i = 0; i < 6; ++i) {
		if (q[i] > params.upper_joints_limits[i])
			BOOST_THROW_EXCEPTION(nfe_joint_limit() << joint_number(i) << limit_type(UPPER_LIMIT));
		else if (q[i] < params.lower_joints_limits[i])
			BOOST_THROW_EXCEPTION(nfe_joint_limit() << joint_number(i) << limit_type(LOWER_LIMIT));
	}
}

void kinematic_model_spkm::check_cartesian_pose(const lib::Homog_matrix& H_) const
{
	// Location of the lower platform A,B, and C points (middle of rotation) - all in relation to the O (global PKM reference frame).
	Homog4d O_lA_T; O_lA_T << 1, 0, 0, 0, 0, 1, 0, params.lA, 0, 0, 1, 0, 0, 0, 0, 1;
	Homog4d O_lB_T; O_lB_T << 1, 0, 0, params.lB, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
	Homog4d O_lC_T; O_lC_T << 1, 0, 0, 0, 0, 1, 0, params.lC, 0, 0, 1, 0, 0, 0, 0, 1;

	// Compute location of the upper platform A,B, and C points (middle of rotation) - all in relation to the O (global PKM reference frame).
	Homog4d uA; uA << 1, 0, 0, 0, 0, 1, 0, params.uA, 0, 0, 1, 0, 0, 0, 0, 1;
	Homog4d uB; uB << 1, 0, 0, params.uB, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
	Homog4d uC; uC << 1, 0, 0, 0, 0, 1, 0, params.uC, 0, 0, 1, 0, 0, 0, 0, 1;

	Homog4d O_uA_T = O_P_T * uA;
	Homog4d O_uB_T = O_P_T * uB;
	Homog4d O_uC_T = O_P_T * uC;

	// Compute angles related to the inner and outer gimbals.
	Homog4d uA_lA_T = O_uA_T - O_lA_T;
	Homog4d uB_lB_T = O_uB_T - O_lB_T;
	Homog4d uC_lC_T = O_uC_T - O_lC_T;

	// Thyk alpha = rotation around the x axes of legs A, B, and C of the upper platform: alpha = arc tan (|y|/|z|).
	double thyk_alpha[3];
	thyk_alpha[0] = atan2 (uA_lA_T(1,3), uA_lA_T(2,3)) * 180.0 / M_PI;
	thyk_alpha[1] = atan2 (uB_lB_T(1,3), uB_lB_T(2,3)) * 180.0 / M_PI;
	thyk_alpha[2] = atan2 (uC_lC_T(1,3), uC_lC_T(2,3)) * 180.0 / M_PI;
#if(DEBUG_KINEMATICS)
	cout << "alpha: A=" << thyk_alpha[0] << " B=" << thyk_alpha[1] << " C=" << thyk_alpha[2] <<endl;
#endif

	// Check thyk alpha angle.
	for (int i = 0; i < 3; ++i) {
		if (thyk_alpha[i] > params.upper_alpha_thyk_angle_limit[i])
			BOOST_THROW_EXCEPTION(nfe_thyk_alpha_limit_exceeded() << angle_number(i) << limit_type(UPPER_LIMIT) << desired_value(thyk_alpha[i]));
		else if (thyk_alpha[i] < params.lower_alpha_thyk_angle_limit[i])
			BOOST_THROW_EXCEPTION(nfe_thyk_alpha_limit_exceeded() << angle_number(i) << limit_type(LOWER_LIMIT) << desired_value(thyk_alpha[i]));
	}

	// Thyk beta = rotation around the y axes of legs A, B, and C of the upper platform: alpha = arc tan (|x|/|z|).
	double thyk_beta[3];
	thyk_beta[0] = atan2 (uA_lA_T(0,3), uA_lA_T(2,3)) * 180.0 / M_PI;
	thyk_beta[1] = atan2 (uB_lB_T(0,3), uB_lB_T(2,3)) * 180.0 / M_PI;
	thyk_beta[2] = atan2 (uC_lC_T(0,3), uC_lC_T(2,3)) * 180.0 / M_PI;
#if(DEBUG_KINEMATICS)
	cout << "beta: A=" << thyk_beta[0] << " B=" << thyk_beta[1] << " C=" << thyk_beta[2] <<endl;
#endif

	// Check thyk beta angle.
	for (int i = 0; i < 3; ++i) {
		if (thyk_beta[i] > params.upper_beta_thyk_angle_limit[i])
			BOOST_THROW_EXCEPTION(nfe_thyk_beta_limit_exceeded() << angle_number(i) << limit_type(UPPER_LIMIT) << desired_value(thyk_beta[i]));
		else if (thyk_beta[i] < params.lower_beta_thyk_angle_limit[i])
			BOOST_THROW_EXCEPTION(nfe_thyk_beta_limit_exceeded() << angle_number(i) << limit_type(LOWER_LIMIT) << desired_value(thyk_beta[i]));
	}

}


void kinematic_model_spkm::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints)
{
	// Compute motor positions for given joints.
	for (int i = 0; i < 6; ++i) {
		local_desired_motor_pos_new[i] = (params.synchro_positions[i] - local_desired_joints[i])
				/ params.mp2i_ratios[i];

		// Round to integer, which is the default motor encoder precision.
		local_desired_motor_pos_new[i] = rint(local_desired_motor_pos_new[i]);
	}
}

void kinematic_model_spkm::mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{
	// Compute joint position basing on motor settings.
	for (int i = 0; i < 6; ++i) {
		local_current_joints[i] = params.synchro_positions[i] - local_current_motor_pos[i] * params.mp2i_ratios[i];
	}
}


void kinematic_model_spkm::inverse_kinematics_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{
	// Transform Homog_matrix to Matrix4d.
	Homog4d O_W_T_desired;
	O_W_T_desired << local_desired_end_effector_frame(0, 0), local_desired_end_effector_frame(0, 1), local_desired_end_effector_frame(0, 2), local_desired_end_effector_frame(0, 3), local_desired_end_effector_frame(1, 0), local_desired_end_effector_frame(1, 1), local_desired_end_effector_frame(1, 2), local_desired_end_effector_frame(1, 3), local_desired_end_effector_frame(2, 0), local_desired_end_effector_frame(2, 1), local_desired_end_effector_frame(2, 2), local_desired_end_effector_frame(2, 3), 0, 0, 0, 1;
#if(DEBUG_KINEMATICS)
    std::cout <<"Desired pose of the end-effector:\n" << O_W_T_desired<<std::endl;
#endif

    // Compute the required O_S_T - pose of the spherical wrist middle (S) in global reference frame (O).
	Homog4d  O_S_T_desired = O_W_T_desired * params.W_S_T;
#if(DEBUG_KINEMATICS)
	std::cout <<"Desired pose of the wrist center:\n" << O_S_T_desired<<std::endl;
#endif

    // Compute e basing only on translation O_S_P from O_S_T.
	Vector5d e = PM_S_to_e(O_S_T_desired);

	// Compute inverse PM kinematics basing on e.
	Vector3d PKM_joints = PM_inverse_from_e(e);

	// Compute upper platform pose.
	O_P_T = PM_O_P_T_from_e(e);
#if(DEBUG_KINEMATICS)
	std::cout <<"Computed upper platform pose:\n" << O_P_T << std::endl;
#endif

	// Compute the pose of wrist (S) on the base of upper platform pose.
	Homog4d O_S_T_computed;
	O_S_T_computed = O_P_T * params.P_S_T;
#if(DEBUG_KINEMATICS)
	std::cout <<"Computed pose of wrist center:\n" << O_S_T_computed << std::endl;
#endif

	// Compute the desired "twist of the wrist".
	// Transformation from computed OST to desired OST.
	Homog4d wrist_twist = O_S_T_computed.inverse()*O_S_T_desired;
#if(DEBUG_KINEMATICS)
	std::cout <<"Twist:\n" << wrist_twist << std::endl;
#endif

	// Compute the inverse transform of the spherical wrist basing on its "twist".
	Vector3d SW_thetas = SW_inverse(wrist_twist, local_current_joints);

	// Fill joints array.
	 local_desired_joints[0] = PKM_joints[0];
	 local_desired_joints[1] = PKM_joints[1];
	 local_desired_joints[2] = PKM_joints[2];
	 local_desired_joints[3] = SW_thetas[0];
	 local_desired_joints[4] = SW_thetas[1];
	 local_desired_joints[5] = SW_thetas[2];
}

Vector5d kinematic_model_spkm::PM_S_to_e(const Homog4d & O_S_T_)
{
	// Extract variables describing position of the middle of wrist
	double x = O_S_T_(0 ,3);
	double y = O_S_T_(1,3);
	double z = O_S_T_(2,3);

#if(DEBUG_KINEMATICS)
	std::cout<<"S= ["<<x<<", "<<y<<", "<<z<<"]\n";
#endif

	// Temporary variables used for computations of alpha.
	double t0_sq = x * x + z * z;
	double hx_sq = params.P_S_P.x() * params.P_S_P.x();

	// Compute sine and cosine of the alpha angle.
	double s_alpha = (z * sqrt(t0_sq - hx_sq) + x * params.P_S_P.x()) / (t0_sq);
	double c_alpha = (-x * sqrt(t0_sq - hx_sq) + z * params.P_S_P.x()) / (t0_sq);

	// Compute sine and cosine of the beta angle.
	double t6 = ((t0_sq - params.lB * x) * sqrt(t0_sq - hx_sq) + params.lB * z * params.P_S_P.x()) / (t0_sq);
	double s_beta = -y / hypot(t6, y); //sqrt(t6 * t6 + y * y);
	double c_beta = t6 / hypot(t6, y); //sqrt(t6 * t6 + y * y);

	// Compute h.
//	double h = (y * y + t6 * sqrt(t0_sq - hx_sq)) / sqrt(t6 * t6 + y * y) - params.P_S_P.z();
	double h = (y * y + t6 * sqrt(t0_sq - hx_sq)) / hypot(t6, y) - params.P_S_P.z();

	// Return computed e vector.*/
	Vector5d e;
	e << s_alpha, c_alpha, s_beta, c_beta, h;

#if(DEBUG_KINEMATICS)
	std::cout<<"e= ["<<e.transpose()<<"]\n";
#endif

	return e;
}

Vector3d kinematic_model_spkm::PM_inverse_from_e(const Vector5d & e_)
{
	// "Retrieve" values from e.
	double s_alpha = e_[0];
	double c_alpha = e_[1];
	double s_beta = e_[2];
	double c_beta = e_[3];
	double h = e_[4];

	// Compute temporary variables.
	double t1 = params.lB * s_alpha - params.uB;
	double t2 = params.lB * c_alpha * c_beta + h;
	double t3 = params.lB * c_alpha - t2 * c_beta;
	// Compute joints.
	double qB = sqrt(t1 * t1 + t2 * t2);
	double qA = sqrt(pow(t3 - params.uA * s_beta, 2.0) + pow(t2 * s_beta - params.uA * c_beta + params.lA, 2.0));
	double qC = sqrt(pow(t3 - params.uC * s_beta, 2.0) + pow(t2 * s_beta - params.uC * c_beta + params.lC, 2.0));

	// Return vector with joints.*/
	Vector3d joints;
	joints << qA, qB, qC;

#if(DEBUG_KINEMATICS)
	std::cout<<"PM joints= ["<<joints.transpose()<<"]\n";
#endif

	return joints;
}

Homog4d kinematic_model_spkm::PM_O_P_T_from_e(const Vector5d & e_)
{
	Homog4d O_P_T;

	// "Retrieve" values from e.
	double s_alpha = e_[0];
	double c_alpha = e_[1];
	double s_beta = e_[2];
	double c_beta = e_[3];
	double h = e_[4];

    // Compute matrix representing the location and orientation of upper platform.
    // Rotation matrix.
    O_P_T(0, 0) = s_alpha;
    O_P_T(0, 1) = -s_beta*c_alpha;
    O_P_T(0, 2) = -c_beta*c_alpha;
    O_P_T(1, 0) = 0;
    O_P_T(1, 1) = c_beta;
    O_P_T(1, 2) = -s_beta;
    O_P_T(2, 0) = c_alpha;
    O_P_T(2, 1) = s_beta*s_alpha;
    O_P_T(2, 2) = c_beta*s_alpha;
    // Translation matrix.
    O_P_T(0, 3) = c_alpha*c_alpha * s_beta*s_beta * params.lB - h*c_beta*c_alpha;
    O_P_T(1, 3) = -c_alpha * s_beta * c_beta * params.lB - h*s_beta;
    O_P_T(2, 3) = -s_beta*s_beta * s_alpha * c_alpha * params.lB + s_alpha*c_beta*h;
    // Last row.
    O_P_T(3, 0) = 0.0;
    O_P_T(3, 1) = 0.0;
    O_P_T(3, 2) = 0.0;
    O_P_T(3, 3) = 1.0;

	// Return matrix.
	return O_P_T;
}

Vector3d kinematic_model_spkm::SW_inverse(const Homog4d & wrist_twist_, const lib::JointArray & local_current_joints)
{
	double phi, theta, psi, dist;
	double phi2, theta2, psi2, dist2;
	Vector3d thetas;

#if(DEBUG_KINEMATICS)
		std::cout.precision(15);
		std::cout<<"u33 = "<< wrist_twist_(2,2) << endl;
#endif

	if ((wrist_twist_(2,2) < (1 + EPS)) && (wrist_twist_(2,2) > (1 - EPS))) {
		// If u33 = 1 then theta is 0.
		theta = 0;
		// Infinite number of solutions: only the phi + psi value can be computed, thus we assume, that phi will equal to the previous one.
		phi = local_current_joints[3];
		psi = atan2(wrist_twist_(1,0), wrist_twist_(0,0)) - phi;
		// atan2(r(2,1), r(1,1)) - phi
#if(DEBUG_KINEMATICS)
		std::cout.precision(15);
		std::cout<<"CASE I: u33=1 => ["<<phi<<", "<<theta<<", "<<psi<<"]\n";
#endif
		thetas << phi, theta, psi;
	} else if ((wrist_twist_(2,2) < (-1 + EPS)) && (wrist_twist_(2,2) > (-1 - EPS))) {
		// If u33 = -1 then theta is equal to pi.
		theta = M_PI;
		// Infinite number of solutions: only the phi - psi value can be computed, thus we assume, that phi will equal to the previous one.
		phi = local_current_joints[3];
		psi = - atan2(-wrist_twist_(0,1), -wrist_twist_(0,0)) + phi;
#if(DEBUG_KINEMATICS)
		std::cout.precision(15);
		std::cout<<"CASE II: u33=-1 => ["<<phi<<", "<<theta<<", "<<psi<<"]\n";
#endif
		thetas << phi, theta, psi;
	} else {
		// Two possible solutions.
//		double sb = hypot(wrist_twist_(2,0), wrist_twist_(2,1));

		// First solution.
		theta = atan2(sqrt(1 - wrist_twist_(2,2)*wrist_twist_(2,2)), wrist_twist_(2,2));
//		theta = atan2(sb, wrist_twist_(2,2));

		phi = atan2(wrist_twist_(1,2), wrist_twist_(0,2));
		psi = atan2(wrist_twist_(2,1), -wrist_twist_(2,0));
#if(DEBUG_KINEMATICS)
		std::cout.precision(15);
		std::cout<<"CASE III: atan(u33, sqrt(1-u33^3)) => ["<<phi<<", "<<theta<<", "<<psi<<"]\n";
#endif
		// Compute maximal delta.
		dist = std::max(std::max(fabs(phi - local_current_joints[3]), fabs(theta - local_current_joints[4])), fabs(psi - local_current_joints[5]));

		// Second solution.
		theta2 = atan2(-sqrt(1 - wrist_twist_(2,2)*wrist_twist_(2,2)), wrist_twist_(2,2));
//		theta = atan2(-sb, wrist_twist_(2,2));

		phi2 = atan2(-wrist_twist_(1,2), -wrist_twist_(0,2));
		psi2 = atan2(-wrist_twist_(2,1), wrist_twist_(2,0));
#if(DEBUG_KINEMATICS)
		std::cout.precision(15);
		std::cout<<"CASE IV: atan(u33, -sqrt(1-u33^3)) => ["<<phi2<<", "<<theta2<<", "<<psi2<<"]\n";
#endif
		// Compute maximal delta.
		dist2 = std::max(std::max(fabs(phi2 - local_current_joints[3]), fabs(theta2 - local_current_joints[4])), fabs(psi2 - local_current_joints[5]));

		// Select best solution.
		if (dist < dist2)
			thetas << phi, theta, psi;
		else
			thetas << phi2, theta2, psi2;
	}

	// Return vector with thetas.
#if(DEBUG_KINEMATICS)
	std::cout<<"SW thetas= ["<<thetas.transpose()<<"]\n";
#endif
	return thetas;
}

} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp
