#if !defined(_BIRD_HAND_CONST_H)
#define _BIRD_HAND_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for Bird Hand three finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup bird_hand
 */

#include "dp_bird_hand.h"

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace bird_hand {

/*!
 * @brief Bird Hand robot label
 * @ingroup bird_hand
 */
const robot_name_t ROBOT_NAME = "ROBOT_BIRD_HAND";

/*!
 * @brief Bird Hand EDP command buffer
 * @ingroup bird_hand
 */
struct cbuffer
{
	/*!
	 * @brief Motion command
	 */
	struct
	{
		/*!
		 * @brief Motion duration in EDP steps
		 */
		int motion_steps;
		/*!
		 * @brief EDP query step number
		 */
		int ecp_query_step;
		single_joint_command finger[NUM_OF_SERVOS];
	} command_structure;
	/*!
	 * @brief Configuration command
	 */
	struct
	{
		single_joint_configuration finger[NUM_OF_SERVOS];
	} configuration_command_structure;
};

/*!
 * @brief Bird Hand EDP reply buffer
 * @ingroup bird_hand
 */
struct rbuffer
{
	struct
	{
		single_joint_status finger[NUM_OF_SERVOS];
	} status_reply_structure;
	struct
	{
		single_joint_configuration finger[NUM_OF_SERVOS];
	} configuration_reply_structure;
}__attribute__((__packed__));

/*!
 * @brief configuration file EDP Bird Hand section string
 * @ingroup bird_hand
 */
const std::string EDP_SECTION = "[edp_bird_hand]";

/*!
 * @brief configuration file ECP Bird Hand section string
 * @ingroup bird_hand
 */
const std::string ECP_SECTION = "[ecp_bird_hand]";

} // namespace bird_hand
} // namespace lib
} // namespace mrrocpp

#endif /* _BIRD_HAND_CONST_H */
