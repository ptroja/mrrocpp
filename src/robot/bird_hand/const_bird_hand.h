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

const robot_name_t ROBOT_BIRD_HAND = "ROBOT_BIRD_HAND"; // three finger Krzysztof Mianowski gripper 2010

struct cbuffer
{
	struct
	{
		int motion_steps;
		int ecp_query_step;
		single_joint_command finger[BIRD_HAND_NUM_OF_SERVOS];
	} command_structure;
	struct
	{
		single_joint_configuration finger[BIRD_HAND_NUM_OF_SERVOS];
	} configuration_command_structure;
};

struct rbuffer
{
	struct
	{
		single_joint_status finger[BIRD_HAND_NUM_OF_SERVOS];
	} status_reply_structure;
	struct
	{
		single_joint_configuration finger[BIRD_HAND_NUM_OF_SERVOS];
	} configuration_reply_structure;
}__attribute__((__packed__));

#define EDP_BIRD_HAND_SECTION "[edp_bird_hand]"
#define ECP_BIRD_HAND_SECTION "[ecp_bird_hand]"

} // namespace bird_hand
} // namespace lib
} // namespace mrrocpp

#endif /* _BIRD_HAND_CONST_H */
