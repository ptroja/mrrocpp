/*!
 * \file kinematic_parameters_bird_hand.cc
 * \brief File containing definitions of kinematic_parameters class methods.
 *
 * \author tkornuta
 * \date Jan 5, 2010
 */

#include "kinematic_parameters_bird_hand.h"

namespace mrrocpp {
namespace kinematics {
namespace bird_hand {

kinematic_parameters_bird_hand::kinematic_parameters_bird_hand() {
	// TODO Set parameters.
	// TODO Set W_S_P;
	// std::cout<< W_S_P<<std::endl;
	// Construct W_S_T on the base of W_S_P.
	//W_S_T.matrix() << Matrix3d::Identity(), W_S_P.transpose(), 0,0,0, 1;
}

} // namespace bird_hand
} // namespace kinematic
} // namespace mrrocpp
