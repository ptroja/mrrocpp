/*!
 * @file
 * @brief File containing the declaration of kinematic_parameters class.
 *
 * @author kczajkowski
 * @author tkornuta
 * @date May 28, 2010
 *
 * @ingroup IRP6OT_KINEMATICS bird_hand
 */

#ifndef KINEMATIC_PARAMETERS_BIRD_HAND_H_
#define KINEMATIC_PARAMETERS_BIRD_HAND_H_

#include "base/lib/typedefs.h"

namespace mrrocpp {
namespace kinematics {
namespace bird_hand {

/*!
 * @struct kinematic_parameters_bird_hand
 * @brief Class storing parameters.
 *
 * @author kczajkowski
 * @date May 28, 2010
 *
 * @ingroup KINEMATICS IRP6OT_KINEMATICS
 */
struct kinematic_parameters_bird_hand {
public:
	//! Constructor - sets the values of the BIRD_HAND geometric parameters.
	kinematic_parameters_bird_hand();

	//! Table storing gear ratio for all DOFs.
	double gear[8];

	//! Lower limits of motor movement.
	double lower_limit_axis[8];

	//! Upper limits of motor movement.
	double upper_limit_axis[8];

	//! Lower limit of joint movement (in radians).
	double lower_limit_joint[8];

	//! Upper limit of joint movement (in radians).
	double upper_limit_joint[8];

	//! Synchronization positions of each joint - in internal coordinates.
	double synchro_joint_position[8];
};

} // namespace bird_hand
} // namespace kinematic
} // namespace mrrocpp

#endif /* KINEMATIC_PARAMETERS_H_ */
