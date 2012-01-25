#if !defined(_SPKM_CONST_H)
#define _SPKM_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace spkm {

/*!
 * @brief SwarmItFix Parallel Kinematic Machine number of motors.
 *
 * The kinematics, as well as control of the whole PKM, is solved for 6DOF - three for PM and three for SW .
 *
 * @ingroup spkm
 */
const int NUM_OF_SERVOS = 6;

/*!
 * @brief Number of segments making up the whole PKM motion.
 *
 *
 * @author Tomasz Kornuta
 * @ingroup spkm
 */
const unsigned int NUM_OF_MOTION_SEGMENTS = 5;

} // namespace spkm
} // namespace lib
} // namespace mrrocpp

#endif /* _SPKM_CONST_H */
