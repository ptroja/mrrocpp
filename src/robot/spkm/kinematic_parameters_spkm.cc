/*!
 * @file
 * @brief File containing definitions of kinematic_parameters class methods.
 *
 * @author tkornuta
 * @date Jan 5, 2010
 *
 * @ingroup SIF_KINEMATICS spkm
 */

#include "kinematic_parameters_spkm.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm {

// Initialization of parameters describing the synchronisation positions of first three parallel PM axes (A=0,B=1,C=2).
const double kinematic_parameters_spkm::synchro_positions[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { 0.252, 0.251, 0.272, 0.0, 0.0, 0.0};

// Initialization of parameters related to conversion from motor positions to joints.
// Parameters for conversion of linear DOF are:
// * Lead roller screw is equal tp 5mm.
// * The encoder has 500 CPT (Counts per turn).
// * Quadcounts = 4 x Encoder Counts.
// * The gear ratio is equal to 9.
const double kinematic_parameters_spkm::mp2i_ratios[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { 0.005 / (4 * 500 * 9), 0.005 / (4 * 500 * 9), 0.005 / (4 * 500 * 9), 0.0, 0.0, 0.0};

// Initialization of upper motors limits vector.
// Those values were computed on the base of "safe" joint limits.
const double kinematic_parameters_spkm::upper_motor_pos_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { 9720, 10800, 8640, 0, 0, 0 };

// Initialization of lower motors limits vector.
// The "unsafe" (in terms that robot can hit its "shell" from inside) are { -194000, -281000, -173000}
// Those values were computed on the base of "safe" joint limits.
const double kinematic_parameters_spkm::lower_motor_pos_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { -100800, -248400, -100799, 0, 0, 0 };

// Initialization of upper joints vector.
// Those are the "safe" limits, not related to sychronization sensors positions.
const double kinematic_parameters_spkm::upper_joints_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { 0.28, 0.32, 0.30, 0, 0, 0 };

// Initialization of lower joints limits vector.
// The lower values are related to positions of synchronization sensors.
const double kinematic_parameters_spkm::lower_joints_limits[mrrocpp::lib::spkm::NUM_OF_SERVOS] = { 0.2493, 0.2480, 0.2696, 0, 0, 0 };

// Lower platform: Initialize the jb coordinate of P1A in O(ib,jb,kb).
const double kinematic_parameters_spkm::dA = -0.05;

// Lower platform: Initialize the ib coordinate of P1B in O(ib,jb,kb).
const double kinematic_parameters_spkm::dB = 0.18;

// Lower platform: Initialize the jb coordinate of P1C in O(ib,jb,kb).
const double kinematic_parameters_spkm::dC = 0.5;

// Upper platform: Initialize the j coordinate of P4A in P(ijk).
const double kinematic_parameters_spkm::pA = -0.05;

// Upper platform: Initialize the i coordinate of P5B in P(ijk).
const double kinematic_parameters_spkm::pB = 0.086;

// Upper platform: Initialize the j coordinate of P4C in P(ijk).
const double kinematic_parameters_spkm::pC = 0.05;

//Initialization of vector representing a translation from P (middle of upper P platform) and S (middle of the spherical wrist).
const Vector3d kinematic_parameters_spkm::P_S_P(0, 0, 0.0905);

//const Vector3d kinematic_parameters_spkm::P_S_P(0, 0, 0.0905);



} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp
