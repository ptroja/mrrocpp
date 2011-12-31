/*!
 * @file
 * @brief File contains dp_spkm class definition for SwarmItFix pkm
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include <cmath>
#include <cstring>
#include <ostream>

#include "dp_spkm.h"

namespace mrrocpp {
namespace lib {
namespace spkm {

//! Constructor with reasonable defaults
_segment::_segment(const lib::Homog_matrix & _goal) :
		goal_pose(_goal), motion_type(lib::epos::SYNC_TRAPEZOIDAL), duration(0), guarded_motion(false)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

