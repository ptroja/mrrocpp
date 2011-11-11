#if !defined(__SBENCH_DATA_PORT_H)
#define __SBENCH_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include <string>

namespace mrrocpp {
namespace lib {
namespace sbench {

/*!
 * @brief SwarmItFix Head total number of pins
 * @ingroup sbench
 */
const int NUM_OF_PINS = 64;

/*!
 * @brief SwarmItFix bench pins activation command data port
 * @ingroup sbench
 */
const std::string COMMAND_DATA_PORT = "SBENCH_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix sbench status data request port
 * @ingroup sbench
 */
const std::string REPLY_DATA_REQUEST_PORT = "SBENCH_REPLY_DATA_REQUEST_PORT";

/*!
 * @brief SwarmItFix bench pins state typedef
 * @ingroup sbench
 */
typedef bool pins_state_td[NUM_OF_PINS];

} // namespace sbench
}
}

#endif
