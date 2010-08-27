#if !defined(_BIRD_HAND_CONST_H)
#define _BIRD_HAND_CONST_H

/*!
 * @file bird_hand_const.h
 * @brief File contains constants and structures for Bird Hand three finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup bird_hand
 */

#include "dp_bird_hand.h"

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {

const robot_name_t ROBOT_BIRD_HAND = "ROBOT_BIRD_HAND"; // three finger Krzysztof Mianowski gripper 2010

struct bird_hand_cbuffer
{
	struct
	{
		int motion_steps;
		int ecp_query_step;
		bird_hand_single_joint_command finger[BIRD_HAND_NUM_OF_SERVOS];
	} bird_hand_command_structure;
	struct
	{
		bird_hand_single_joint_configuration finger[BIRD_HAND_NUM_OF_SERVOS];
	} bird_hand_configuration_command_structure;
};

struct bird_hand_rbuffer
{
	struct
	{
		bird_hand_single_joint_status finger[BIRD_HAND_NUM_OF_SERVOS];
	} bird_hand_status_reply_structure;
	struct
	{
		bird_hand_single_joint_configuration finger[BIRD_HAND_NUM_OF_SERVOS];
	} bird_hand_configuration_reply_structure;
}__attribute__((__packed__));

#define EDP_BIRD_HAND_SECTION "[edp_bird_hand]"
#define ECP_BIRD_HAND_SECTION "[ecp_bird_hand]"

} // namespace lib
} // namespace mrrocpp

#endif /* _BIRD_HAND_CONST_H */
