/*!
 * \file kinematic_model_spkm.cc
 * \brief File containing definition of kinematic_model_spkm class methods.
 *
 * \author tkornuta
 * \date Jan 05, 2010
 */

#include <math.h>

#include "lib/com_buf.h"
#include "kinematics/spkm/kinematic_model_spkm.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm {

kinematic_model_spkm::kinematic_model_spkm(void)
{
	// Set model name.
	set_kinematic_model_label("SPKM kinematic model by D.Zlatanow and M.Zoppi");
}

void kinematic_model_spkm::check_motor_position(const lib::MotorArray & motor_position)
{
}

void kinematic_model_spkm::check_joints(const lib::JointArray & q)
{
}

void kinematic_model_spkm::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, lib::JointArray & local_desired_joints)
{
}

void kinematic_model_spkm::inverse_kinematics_transform(lib::JointArray & local_desired_joints, lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{
	// Transform Homog_matrix to Matrix4d.
	Homog4d O_W_T;
	O_W_T.matrix() << local_desired_end_effector_frame(0,0),  local_desired_end_effector_frame(0,1), local_desired_end_effector_frame(0,2), local_desired_end_effector_frame(0,3),
			local_desired_end_effector_frame(1,0),  local_desired_end_effector_frame(1,1), local_desired_end_effector_frame(1,2), local_desired_end_effector_frame(1,3),
			local_desired_end_effector_frame(2,0),  local_desired_end_effector_frame(2,1), local_desired_end_effector_frame(2,2), local_desired_end_effector_frame(2,3),
			0, 0, 0, 1;

	// Compute O_S_T.
	Homog4d O_S_T = O_W_T*params.W_S_T;
	// Extract translation O_S_P.
	Vector3d O_S_P = O_S_T.translation();

	// Compute e basing only on translation O_S_P from O_S_T.
	Vector5d e = PKM_S_to_e(O_S_P);
	// Compute inverse PKM kinematics basing on e.
	Vector3d PKM_joints = PKM_inverse_from_e(e);

	// Compute upper platform pose.
	Homog4d O_P_T = PKM_O_P_T_from_e(e);
	// Compute location of wrist end-effector in relation to its base (upper PKM platform).
	Homog4d P_W_T;
	P_W_T.matrix() = O_P_T.inverse(Isometry) * O_W_T;

	// Compute spherical wrist inverse kinematics.
	Vector3d SW_joints = SW_inverse(P_W_T);

	// Fill joints array.
	local_current_joints[0] = PKM_joints[0];
	local_current_joints[1] = PKM_joints[1];
	local_current_joints[2] = PKM_joints[2];
	local_current_joints[3] = SW_joints[0];
	local_current_joints[4] = SW_joints[1];
	local_current_joints[5] = SW_joints[2];
}

Vector5d kinematic_model_spkm::PKM_S_to_e(Vector3d _O_S_P)
{
	// Temporary variables used for computations of alpha..
	double t0_sq = _O_S_P.x() * _O_S_P.x() + _O_S_P.z() * _O_S_P.z();
	double hx_sq = params.S_P_P.x() * params.S_P_P.x();

	// Compute sine and cosine of the alpha angle.
	double s_alpha = (params.delta_B1 * _O_S_P.z() * sqrt(t0_sq - hx_sq) + _O_S_P.x() * params.S_P_P.x()) / (t0_sq);
	double c_alpha = (-params.delta_B1 * _O_S_P.x() * sqrt(t0_sq - hx_sq) + _O_S_P.z() * params.S_P_P.x()) / (t0_sq);

	// Compute sine and cosine of the beta angle.
	double t6 = (params.delta_B1 * (_O_S_P.z() * _O_S_P.z() + _O_S_P.x() * _O_S_P.x() - params.dB * _O_S_P.x())
			* sqrt(t0_sq - hx_sq) + params.dB * _O_S_P.z() * params.S_P_P.x()) / (t0_sq);
	double s_beta = -params.delta_B2 * _O_S_P.y() / sqrt(t6 * t6 + _O_S_P.y() * _O_S_P.y());
	double c_beta = params.delta_B2 * t6 / sqrt(t6 * t6 + _O_S_P.y() * _O_S_P.y());

	// Compute h.
	double h = params.delta_B2 * (_O_S_P.y() * _O_S_P.y() + params.delta_B1 * t6 * sqrt(t0_sq - hx_sq)) / sqrt(t6 * t6
			+ _O_S_P.y() * _O_S_P.y()) - params.S_P_P.z();

	// Return computed e vector.
	Vector5d e;
	e << s_alpha, c_alpha, s_beta, c_beta, h;
	return e;
}

Vector3d kinematic_model_spkm::PKM_inverse_from_e(Vector5d _e)
{
	// "Retrieve" values from e.
	double s_alpha = _e[0];
	double c_alpha = _e[1];
	double s_beta = _e[2];
	double c_beta = _e[3];
	double h = _e[4];

	// Compute temporary variables.
	double t1 = params.dB * s_alpha - params.pB;
	double t2 = params.dB * c_alpha * c_beta + h;
	double t3 = params.dB * c_alpha - t2 * c_beta;
	// Compute joints.
	double qB = sqrt(t1 * t1 + t2 * t2);
	double qA = sqrt(pow(t3 - params.pB * s_beta - params.hA * c_beta + params.delta_A * params.l12A, 2.0) + pow(t2
			* s_beta - params.pB * c_beta + params.hA * s_beta + params.dA, 2.0));
	double qC = sqrt(pow(t3 - params.pC * s_beta - params.hC * c_beta + params.delta_C * params.l12C, 2.0) + pow(t2
			* s_beta - params.pC * c_beta + params.hC * s_beta + params.dC, 2.0));

	// Return vector with joints.
	Vector3d joints;
	joints << qA, qB, qC;
	return joints;
}

Homog4d kinematic_model_spkm::PKM_O_P_T_from_e(Vector5d _e)
{
	Matrix4d O_P_T;

	// "Retrieve" values from e.
	double s_alpha = _e[0];
	double c_alpha = _e[1];
	double s_beta = _e[2];
	double c_beta = _e[3];
	double h = _e[4];

	// Compute matrix representing the location and orientation of upper platform.
	O_P_T(0,0) = s_alpha;
	O_P_T(0,1) = 0.0;
	O_P_T(0,2) = c_alpha;
	O_P_T(0,3) = 0.0;
	O_P_T(1,0) = -s_beta * c_alpha;
	O_P_T(1,1) = c_beta;
	O_P_T(1,2) = s_beta * s_alpha;
	O_P_T(1,3) = c_alpha * params.dB * s_beta;
	O_P_T(2,0) = -c_beta * c_alpha;
	O_P_T(2,1) = -s_beta;
	O_P_T(2,2) = c_beta * s_alpha;
	O_P_T(2,3) = -h;
	O_P_T(3,0) = 0.0;
	O_P_T(3,1) = 0.0;
	O_P_T(3,2) = 0.0;
	O_P_T(3,3) = 1.0;

	// Return matrix.
	return Homog4d(O_P_T);
}

Vector3d kinematic_model_spkm::SW_inverse(Homog4d _P_W_T) {
	// TODO Inverse kinematics of the spherical wrist.

	// Return vector with joints.
	Vector3d joints;
	joints << 0, 0, 0;
	return joints;

}



} // namespace spkm
} // namespace kinematic
}
// namespace mrrocpp

