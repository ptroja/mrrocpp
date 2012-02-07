/*!
 * @file kinematic_parameters_spkm.cpp
 * @brief File containing the initialization of parameters of both PKM agenta.
 *
 * @author Tomasz Kornuta
 * @date Jan 21, 01 2012
 * @ingroup SIF_KINEMATICS spkm
*/

#include "kinematic_parameters_spkm.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm {

kinematic_parameters_spkm::kinematic_parameters_spkm()
{

	// Value  of thyk alpha angle for legs A and C (the same for both sides).
	alpha_thyk_angle_limit_AC = 30;

	// Value  of thyk alpha internal angle for leg B (the same for both sides).
	alpha_thyk_angle_limit_B_int = 20;

	// Value  of thyk alpha external angle for leg B (the same for both sides).
	alpha_thyk_angle_limit_B_ext = 30;

	// Value  of thyk beta internal angle for leg B (the same for both sides).
	beta_thyk_angle_limit_B_int= -29;

	// Value  of thyk beta external angle for leg B (the same for both sides).
	beta_thyk_angle_limit_B_ext = -37;
}

}

}

}
