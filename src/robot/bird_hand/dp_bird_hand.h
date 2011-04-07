#if !defined(__BIRD_HAND_DATA_PORT_H)
#define __BIRD_HAND_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for Bird Hand three finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup bird_hand
 */

namespace mrrocpp {
namespace lib {
namespace bird_hand {

/*!
 * @brief Bird Hand motion command data port
 * @ingroup bird_hand
 */
const std::string COMMAND_DATA_PORT = "bird_hand_command_data_port";
/*!
 * @brief Bird Hand motion status data request port
 * @ingroup bird_hand
 */
const std::string STATUS_DATA_REQUEST_PORT = "bird_hand_status_data_request_port";

/*!
 * @brief Bird Hand configuration command data port
 * @ingroup bird_hand
 */
const std::string CONFIGURATION_DATA_PORT = "bird_hand_configuration_data_port";
/*!
 * @brief Bird Hand configuration status data request port
 * @ingroup bird_hand
 */
const std::string CONFIGURATION_DATA_REQUEST_PORT = "bird_hand_configuration_data_request_port";

/*!
 * @brief Bird Hand total number of servos
 * @ingroup bird_hand
 */
const int NUM_OF_SERVOS = 8;
/*!
 * @brief Bird Hand thumb finger number of servos
 * @ingroup bird_hand
 */
const int THUMB_F_NUM_OF_SERVOS = 2;
/*!
 * @brief Bird Hand index finger number of servos
 * @ingroup bird_hand
 */
const int INDEX_F_NUM_OF_SERVOS = 3;
/*!
 * @brief Bird Hand ring finger number of servos
 * @ingroup bird_hand
 */
const int RING_F_NUM_OF_SERVOS = 3;

/*!
 * @brief Bird Hand three commanded motion variants enumeration
 * @ingroup bird_hand
 */
enum MOTION_VARIANT
{
	SIGLE_STEP_POSTION_INCREMENT = 0, MACROSTEP_POSITION_INCREMENT = 1, MACROSTEP_ABSOLUTE_POSITION = 2
};

/*!
 * @brief Bird Hand poition/torque command for single joint
 * @ingroup bird_hand
 */
struct single_joint_command
{
	MOTION_VARIANT profile_type;
	double reciprocal_of_damping;
	double desired_torque;
	double desired_position;
}__attribute__((__packed__));

/*!
 * @brief Bird Hand single joint reply status
 * @ingroup bird_hand
 */
struct single_joint_status
{
	double meassured_position;
	double meassured_torque;
	double measured_current;
	bool upper_limit_of_absolute_value_of_desired_position_increment;
	bool upper_limit_of_absolute_value_of_computed_position_increment;
	bool upper_limit_of_absolute_position;
	bool lower_limit_of_absolute_position;
	bool upper_limit_of_absolute_value_of_desired_torque;
	bool lower_limit_of_absolute_value_of_desired_torque;
	bool upper_limit_of_absolute_value_of_meassured_torque;
	bool upper_limit_of_measured_current;
}__attribute__((__packed__));

/*!
 * @brief Bird Hand configuration command for single joint
 * @ingroup bird_hand
 */
struct single_joint_configuration
{
	int p_factor;
	int i_factor;
	int d_factor;
	int value_of_upper_limit_of_absolute_position;
	int value_of_lower_limit_of_absolute_position;
	int value_of_upper_limit_of_measured_current;
	int value_of_upper_limit_of_absolute_value_of_torque;
	int value_of_lower_limit_of_absolute_value_of_torque;
	int value_of_lower_limit_of_absolute_value_of_meassured_torque;
	int value_of_upper_limit_of_position_increment;
}__attribute__((__packed__));

/*!
 * @brief multi joint position/torque command for whole gripper
 * @ingroup bird_hand
 */
struct command
{
	int motion_steps;
	int ecp_query_step;
	single_joint_command thumb_f[THUMB_F_NUM_OF_SERVOS];
	single_joint_command index_f[INDEX_F_NUM_OF_SERVOS];
	single_joint_command ring_f[RING_F_NUM_OF_SERVOS];
}__attribute__((__packed__));

/*!
 * @brief multi joint reply status for whole gripper
 * @ingroup bird_hand
 */
struct status
{
	single_joint_status thumb_f[THUMB_F_NUM_OF_SERVOS];
	single_joint_status index_f[INDEX_F_NUM_OF_SERVOS];
	single_joint_status ring_f[RING_F_NUM_OF_SERVOS];
}__attribute__((__packed__));

/*!
 * @brief multi joint configuration command for whole gripper
 * @ingroup bird_hand
 */
struct configuration
{
	single_joint_configuration thumb_f[THUMB_F_NUM_OF_SERVOS];
	single_joint_configuration index_f[INDEX_F_NUM_OF_SERVOS];
	single_joint_configuration ring_f[RING_F_NUM_OF_SERVOS];
}__attribute__((__packed__));

} // namespace bird_hand
} // namespace lib
} // namespace mrrocpp

#endif
