#if !defined(__MP_TYPEDEFS_H)
#define __MP_TYPEDEFS_H

/*!
 * @file
 * @brief File contains structures used in mp classes
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <map>

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace mp {
namespace robot {
class robot;
}

namespace common {

typedef std::map <lib::robot_name_t, robot::robot*> robots_t;
typedef robots_t::value_type robot_pair_t;

} // namespace common

} // namespace mp
} // namespace mrrocpp

#endif
