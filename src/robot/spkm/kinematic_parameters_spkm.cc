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
