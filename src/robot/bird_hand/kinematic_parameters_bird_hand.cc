/*!
 * \file kinematic_parameters_bird_hand.cc
 * \brief File containing definitions of kinematic_parameters class methods.
 *
 * \author kczajkowski
 * \date May 28, 2010
 */

#include <math.h>

#include "kinematic_parameters_bird_hand.h"

namespace mrrocpp {
namespace kinematics {
namespace bird_hand {

kinematic_parameters_bird_hand::kinematic_parameters_bird_hand() {
	gear[0] = 						  //index_f[2]
	gear[1] = 						  //index_f[1]
	gear[2] = 						  //thumb_f[0]
	gear[3] = 						  //thumb_f[1]
	gear[4] = 						  //ring_f[1]
	gear[5] = 512.0 * 275.0 * 7.826;  //ring_f[2]
	gear[6] = 						  //index_f[0] (rotation)
	gear[7] = 512.0 * 275.0 * 2.1585; //index_f[0] (rotation)

	lower_limit_axis[0] = 460;
	lower_limit_axis[1] = 730;
	lower_limit_axis[2] = 1790;
	lower_limit_axis[3] = 1450;
	lower_limit_axis[4] = 340;
	lower_limit_axis[5] = 1630;
	lower_limit_axis[6] = 0;
	lower_limit_axis[7] = 0;

	upper_limit_axis[0] = 1350;
	upper_limit_axis[1] = 1750;
	upper_limit_axis[2] = 2630;
	upper_limit_axis[3] = 2230;
	upper_limit_axis[4] = 1040;
	upper_limit_axis[5] = 2540;
	upper_limit_axis[6] = 4096;
	upper_limit_axis[7] = 4096;

	lower_limit_joint[0] = 0.0;
	lower_limit_joint[1] = 0.0;
	lower_limit_joint[2] = 0.0;
	lower_limit_joint[3] = 0.0;
	lower_limit_joint[4] = 0.0;
	lower_limit_joint[5] = 0.0;
	lower_limit_joint[6] = 0.0;
	lower_limit_joint[7] = 0.0;

	upper_limit_joint[0] = 60.0 * M_PI / 180.0;
	upper_limit_joint[1] = 60.0 * M_PI / 180.0;
	upper_limit_joint[2] = 60.0 * M_PI / 180.0;
	upper_limit_joint[3] = 60.0 * M_PI / 180.0;
	upper_limit_joint[4] = 60.0 * M_PI / 180.0;
	upper_limit_joint[5] = 60.0 * M_PI / 180.0;
	upper_limit_joint[6] = 90.0 * M_PI / 180.0;
	upper_limit_joint[7] = 90.0 * M_PI / 180.0;

//	synchro_joint_position[0] = -0.15;
//	synchro_joint_position[1] = -0.36;
//	synchro_joint_position[2] = -0.44;
//	synchro_joint_position[3] = -0.55;
//	synchro_joint_position[4] = -0.0;
//	synchro_joint_position[5] = -0.616;
//	synchro_joint_position[6] = -0.616;
//	synchro_joint_position[7] = -0.616;

	synchro_joint_position[0] = -0.0;
	synchro_joint_position[1] = -0.0;
	synchro_joint_position[2] = -0.0;
	synchro_joint_position[3] = -0.0;
	synchro_joint_position[4] = -0.0;
	synchro_joint_position[5] = -0.0;
	synchro_joint_position[6] = -0.0;
	synchro_joint_position[7] = -0.0;

	synchro_motor_position[0] = synchro_joint_position[0] * gear[0];
	synchro_motor_position[1] = synchro_joint_position[1] * gear[1];
	synchro_motor_position[2] = synchro_joint_position[2] * gear[2];
	synchro_motor_position[3] = synchro_joint_position[3] * gear[3];
	synchro_motor_position[4] = synchro_joint_position[4] * gear[4];
	synchro_motor_position[5] = synchro_joint_position[5] * gear[5];
	synchro_motor_position[6] = synchro_joint_position[6] * gear[6];
	synchro_motor_position[7] = synchro_joint_position[7] * gear[7];
}

} // namespace bird_hand
} // namespace kinematic
} // namespace mrrocpp
