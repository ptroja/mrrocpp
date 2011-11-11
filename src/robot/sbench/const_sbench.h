#if !defined(_SBENCH_CONST_H)
#define _SBENCH_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */



#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace sbench {

/*!
 * @brief SwarmItFix Head robot label
 * @ingroup sbench
 */
const robot_name_t ROBOT_NAME = "sbench";

/*!
 * @brief SwarmItFix Head total number of pins
 * @ingroup sbench
 */
const int NUM_OF_PINS = 64;

/*!
 * @brief SwarmItFix Head total number of servos
 * @ingroup sbench
 */
const int NUM_OF_SERVOS = 0;

} // namespace sbench
} // namespace lib
} // namespace mrrocpp

#endif /* _SBENCH_CONST_H */
