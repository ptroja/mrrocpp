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
	gear[0] = 275.0 * 7.826;  //thumb_f 0
	gear[1] = 275.0 * 7.826;  //thumb_f 1
	gear[2] = 275.0 * 2.1585; //index_f rotation
	gear[3] = 275.0 * 7.826;  //index_f 0
	gear[4] = 275.0 * 7.826;  //index_f 1
	gear[5] = 275.0 * 2.1585; //ring_f rotation
	gear[6] = 275.0 * 7.826;  //ring_f 0
	gear[7] = 275.0 * 7.826;  //ring_f 1

	//TODO fix lower limits
	lower_limit_axis[0] = -1000000.0;
	lower_limit_axis[1] = -1000000.0;
	lower_limit_axis[2] = -1000000.0;
	lower_limit_axis[3] = -1000000.0;
	lower_limit_axis[4] = -1000000.0;
	lower_limit_axis[5] = -1000000.0;
	lower_limit_axis[6] = -1000000.0;
	lower_limit_axis[7] = -1000000.0;

	//TODO fix upper limits
	upper_limit_axis[0] = 1000000.0;
	upper_limit_axis[1] = 1000000.0;
	upper_limit_axis[2] = 1000000.0;
	upper_limit_axis[3] = 1000000.0;
	upper_limit_axis[4] = 1000000.0;
	upper_limit_axis[5] = 1000000.0;
	upper_limit_axis[6] = 1000000.0;
	upper_limit_axis[7] = 1000000.0;

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
	upper_limit_joint[2] = 90.0 * M_PI / 180.0;
	upper_limit_joint[3] = 60.0 * M_PI / 180.0;
	upper_limit_joint[4] = 60.0 * M_PI / 180.0;
	upper_limit_joint[5] = 90.0 * M_PI / 180.0;
	upper_limit_joint[6] = 60.0 * M_PI / 180.0;
	upper_limit_joint[7] = 60.0 * M_PI / 180.0;

	//TODO fix synchro
	synchro_motor_position[0] = 0.0;
	synchro_motor_position[1] = 0.0;
	synchro_motor_position[2] = 0.0;
	synchro_motor_position[3] = 0.0;
	synchro_motor_position[4] = 0.0;
	synchro_motor_position[5] = 0.0;
	synchro_motor_position[6] = 0.0;
	synchro_motor_position[7] = 0.0;

	synchro_joint_position[0] = synchro_motor_position[0] / gear[0];
	synchro_joint_position[1] = synchro_motor_position[1] / gear[1];
	synchro_joint_position[2] = synchro_motor_position[2] / gear[2];
	synchro_joint_position[3] = synchro_motor_position[3] / gear[3];
	synchro_joint_position[4] = synchro_motor_position[4] / gear[4];
	synchro_joint_position[5] = synchro_motor_position[5] / gear[5];
	synchro_joint_position[6] = synchro_motor_position[6] / gear[6];
	synchro_joint_position[7] = synchro_motor_position[7] / gear[7];
}

} // namespace bird_hand
} // namespace kinematic
} // namespace mrrocpp
