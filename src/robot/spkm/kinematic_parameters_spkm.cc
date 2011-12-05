/*!
 * @file
 * @brief File containing definitions of kinematic_parameters class methods.
 *
 * @author Tomasz Kornuta
 * @date Jan 5, 2010
 *
 * @ingroup SIF_KINEMATICS spkm
 */

#include "const_spkm.h"
#include "kinematic_parameters_spkm.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm {

//! Initialization of parameters describing the synchronisation positions (in joints).
const double kinematic_parameters_spkm::synchro_positions[mrrocpp::lib::spkm::NUM_OF_SERVOS] = {  0.2405, 0.242, 0.2405, 0.0, 0.0, -0.2906};

//! Initialization of parameters related to conversion from motor positions to joints.
//! Parameters for conversion for linear DOFs are:
//! * Lead roller screw is equal to 5mm.
//! * The encoder has 500 CPT (Counts per turn).
//! * Quadcounts = 4 x Encoder Counts.
//! * The gear ratio is equal to 9.
const double linear_mp2i_ratio = 0.005 / (4 * 500 * 9);
//! Parameters for conversion for rotational DOFs are:
//! * The encoder has 2000 CPT (Counts per turn).
//! * Quadcounts = 4 x Encoder Counts.
//! * The gear ratio is equal to 100.
const double rotational_mp2i_ratio = -2*M_PI / (4 * 2000 * 100);
const double kinematic_parameters_spkm::mp2i_ratios[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { linear_mp2i_ratio, linear_mp2i_ratio, linear_mp2i_ratio, rotational_mp2i_ratio, rotational_mp2i_ratio, rotational_mp2i_ratio };

//! Initialization of the encoder resolution. Equals to the Counts Per Turn (CPT) x 4.
const uint32_t kinematic_parameters_spkm::encoder_resolution[mrrocpp::lib::spkm::NUM_OF_SERVOS] = {
		500*4, 500*4, 500*4,
		2000*4, 2000*4, 2000*4
};

//! Initialization of upper motors limits vector.
const int32_t kinematic_parameters_spkm::upper_motor_pos_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { 8000, 8000, 8000, 320000, 90000, 340000 };

//! Initialization of lower motors limits vector.
//! Those are the "unsafe" (in terms that robot can hit its "shell" from inside) values.
const int32_t kinematic_parameters_spkm::lower_motor_pos_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { -350000, -350000, -350000, -320000, -200000, -280000 };

//! Initialization of upper joints vector.
//! Those are the "safe" limits, not related to synchronization sensors positions.
const double kinematic_parameters_spkm::upper_joints_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { 0.28, 0.292, 0.28, 2.7489, 0.7, 2.6704 };

//! Initialization of lower joints limits vector.
//! The lower values are related to positions of synchronization sensors.
const double kinematic_parameters_spkm::lower_joints_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { 0.242, 0.242, 0.242, -2.2777, -1.5708, -2.4347 };

//! Initialization of upper thyk alpha angle limit.
//! Those values were determined experimentally.
const double kinematic_parameters_spkm::upper_alpha_thyk_angle_limit[3] = { 30.0, 50.0, 30.0 };

//! Initialization of lower thyk alpha angle limit.
//! Those values were determined experimentally.
const double kinematic_parameters_spkm::lower_alpha_thyk_angle_limit[3] = { -30.0, -50.0, -30.0 };

//! Initialization of upper thyk beta angle limit.
//! Those values were determined experimentally.
const double kinematic_parameters_spkm::upper_beta_thyk_angle_limit[3] = { 30.0, 50.0, 30.0 };

//! Initialization of lower thyk beta angle limit.
//! Those values were determined experimentally.
const double kinematic_parameters_spkm::lower_beta_thyk_angle_limit[3] = { -30.0, -50.0, -30.0 };

//! Lower platform: Initialize the jb coordinate of P1A in O(ib,jb,kb).
const double kinematic_parameters_spkm::lA = -0.05;

//! Lower platform: Initialize the ib coordinate of P1B in O(ib,jb,kb).
const double kinematic_parameters_spkm::lB = 0.18;

//! Lower platform: Initialize the jb coordinate of P1C in O(ib,jb,kb).
const double kinematic_parameters_spkm::lC = 0.05;

//! Upper platform: Initialize the j coordinate of P4A in P(ijk).
const double kinematic_parameters_spkm::uA = -0.05;

//! Upper platform: Initialize the i coordinate of P5B in P(ijk).
const double kinematic_parameters_spkm::uB = 0.086;

//! Upper platform: Initialize the j coordinate of P4C in P(ijk).
const double kinematic_parameters_spkm::uC = 0.05;

//! Initialization of vector representing a translation from P (middle of upper P platform) and S (middle of the spherical wrist).
const Vector3d kinematic_parameters_spkm::P_S_P(0, 0, 0.0905);

//! Initialization of transformation from P (middle of upper P platform) and S (middle of the spherical wrist).
const double tmp_pst[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0.0905, 1};
const Homog4d kinematic_parameters_spkm::P_S_T(tmp_pst);

//! Initialization of transformation from W (SW end-effector) to S (middle of the spherical wrist).
//! The W_S_T is the inversion of:
//! S_W_P = [0.0; 0; 0.0725];
//! S_W_R = [1, 0, 0; 0 1 0; 0, 0, 1]
const double tmp_wst[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, -0.0725, 1};
const Homog4d kinematic_parameters_spkm::W_S_T(tmp_wst);



} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp
