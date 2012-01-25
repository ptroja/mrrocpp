#if !defined(_BIRD_HAND_CONST_H)
#define _BIRD_HAND_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for Bird Hand three finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup bird_hand
 */

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace bird_hand {

/*!
 * @brief Bird Hand robot label
 * @ingroup bird_hand
 */
const robot_name_t ROBOT_NAME = "bird_hand";

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

} // namespace bird_hand
} // namespace lib
} // namespace mrrocpp

#endif /* _BIRD_HAND_CONST_H */
