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
const double kinematic_parameters_spkm::synchro_positions[6] = { 0.252, 0.251, 0.272, 0.0, 0.0, 0.0};

// Initialization of parameters related to conversion from motor positions to joints.
// Parameters for conversion of linear DOF are:
// * Lead roller screw is equal tp 5mm.
// * The encoder has 500 CPT (Counts per turn).
// * Quadcounts = 4 x Encoder Counts.
// * The gear ratio is equal to 9.
const double kinematic_parameters_spkm::mp2i_ratios[6] = { 0.005 / (4 * 500 * 9), 0.005 / (4 * 500 * 9), 0.005 / (4 * 500 * 9), 0.0, 0.0, 0.0};

// Initialization of max motors limits vector.
const double kinematic_parameters_spkm::max_motor_pos_limits[6] = { 10998, 11000, 10207, 0, 0, 0 };

// Initialization of min motors limits vector.
const double kinematic_parameters_spkm::min_motor_pos_limits[6] = { -194493, -282203, -174524, 0, 0, 0 };

// Initialization of max joints vector.
const double kinematic_parameters_spkm::max_joints_limits[6] = { 4.6327, 6.6069, 4.2027, 0, 0, 0 };

// Initialization of min joints limits vector.
const double kinematic_parameters_spkm::min_joints_limits[6] = { 0.0043, 0.0033, 0.0421, 0, 0, 0 };



kinematic_parameters_spkm::kinematic_parameters_spkm()
{
  // TODO Set parameters.
	// TODO Set W_S_P;
	// std::cout<< W_S_P<<std::endl;
	// Construct W_S_T on the base of W_S_P.
	//W_S_T.matrix() << Matrix3d::Identity(), W_S_P.transpose(), 0,0,0, 1;
}


} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp
