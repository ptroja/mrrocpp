/*!
 * @file
 * @brief File containing definitions of kinematic_parameters class methods.
 *
 * @author tkornuta
 * @date Jan 5, 2010
 *
 * @ingroup SIF_KINEMATICS spkm
 */

#include "const_spkm.h"
#include "kinematic_parameters_spkm.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm {

// Initialization of parameters describing the synchronisation positions of first three parallel PM axes (A=0,B=1,C=2).
const double kinematic_parameters_spkm::synchro_positions[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { 0.2515, 0.272, 0.2515, 0.0, 0.0, 0.0};
// TODO: nowe wartości po zmianie sposobu synchronizacji -> 0.242, 0.265, 0.242

// Initialization of parameters related to conversion from motor positions to joints.
// Parameters for conversion for linear DOFs are:
// * Lead roller screw is equal to 5mm.
// * The encoder has 500 CPT (Counts per turn).
// * Quadcounts = 4 x Encoder Counts.
// * The gear ratio is equal to 9.
const double linear_mp2i_ratio = 0.005 / (4 * 500 * 9);
// Parameters for conversion for rotational DOFs are:
// * The encoder has 2000 CPT (Counts per turn).
// * Quadcounts = 4 x Encoder Counts.
// * The gear ratio is equal to 100.
const double rotational_mp2i_ratio = 2*M_PI / (4 * 2000 * 100);
const double kinematic_parameters_spkm::mp2i_ratios[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { linear_mp2i_ratio, linear_mp2i_ratio, linear_mp2i_ratio, rotational_mp2i_ratio, rotational_mp2i_ratio, rotational_mp2i_ratio };

// Initialization of the encoder resolution. Equals to the Counts Per Turn (CPT) x 4.
const uint32_t kinematic_parameters_spkm::encoder_resolution[mrrocpp::lib::spkm::NUM_OF_SERVOS] = {
		500*4, 500*4, 500*4,
		2000*4, 2000*4, 2000*4
};

// Initialization of upper motors limits vector.
// Those values were computed on the base of "safe" joint limits.
const int32_t kinematic_parameters_spkm::upper_motor_pos_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { 9810, 10810, 9810, 350010, 60010, 340010 };

// Initialization of lower motors limits vector.
// The "unsafe" (in terms that robot can hit its "shell" from inside) are { -194000, -281000, -173000 }
// Those values were computed on the base of "safe" joint limits.
const int32_t kinematic_parameters_spkm::lower_motor_pos_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { -100810, -183410, -100810, -290010, -70010, -290010 };

// Initialization of upper joints vector.
// Those are the "safe" limits, not related to synchronization sensors positions.
const double kinematic_parameters_spkm::upper_joints_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { 0.2795, 0.32, 0.2795, 2.7489, 0.4712, 2.6704 };

// Initialization of lower joints limits vector.
// The lower values are related to positions of synchronization sensors.
const double kinematic_parameters_spkm::lower_joints_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { 0.2488, 0.2690, 0.2488, -2.2777, -0.5498, -2.4347 };

// Lower platform: Initialize the jb coordinate of P1A in O(ib,jb,kb).
const double kinematic_parameters_spkm::dA = -0.05;

// Lower platform: Initialize the ib coordinate of P1B in O(ib,jb,kb).
const double kinematic_parameters_spkm::dB = 0.18;

// Lower platform: Initialize the jb coordinate of P1C in O(ib,jb,kb).
const double kinematic_parameters_spkm::dC = 0.05;

// Upper platform: Initialize the j coordinate of P4A in P(ijk).
const double kinematic_parameters_spkm::pA = -0.05;

// Upper platform: Initialize the i coordinate of P5B in P(ijk).
const double kinematic_parameters_spkm::pB = 0.086;

// Upper platform: Initialize the j coordinate of P4C in P(ijk).
const double kinematic_parameters_spkm::pC = 0.05;

// Initialization of vector representing a translation from P (middle of upper P platform) and S (middle of the spherical wrist).
const Vector3d kinematic_parameters_spkm::P_S_P(0, 0, 0.0905);

// Initialization of transformation from P (middle of upper P platform) and S (middle of the spherical wrist).
const double tmp_pst[16] = {0, -1.0, 0, 0, 1.0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0.0905, 1};
const Homog4d kinematic_parameters_spkm::P_S_T(tmp_pst);

// Initialization of transformation from W (SW end-effector) to S (middle of the spherical wrist).
// The W_S_T is the inversion of:
// S_W_P = [0.0; 0; 0.0725];
// S_W_R = [1, 0, 0; 0 1 0; 0, 0, 1]
const double tmp_wst[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, -0.0725, 1};
const Homog4d kinematic_parameters_spkm::W_S_T(tmp_wst);



} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp
