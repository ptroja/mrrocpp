#if !defined(_CONST_PLAYER_H)
#define _CONST_PLAYER_H

/*!
 * @file
 * @brief File contains constants and structures for Festival
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup player
 */

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace electron {

const robot_name_t ROBOT_NAME = "ROBOT_ELECTRON";

const std::string ECP_SECTION = "[ecp_electron]";

} // namespace electron

namespace speechrecognition {

const robot_name_t ROBOT_NAME = "ROBOT_SPEECHRECOGNITION";

const std::string ECP_SECTION = "[ecp_speechrecognition]";

} // namespace speechrecognition
} // namespace lib
} // namespace mrrocpp

#endif /* _CONST_PLAYER_H */
