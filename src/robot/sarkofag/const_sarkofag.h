#if !defined(_SARKOFAG_CONST_H)
#define _SARKOFAG_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for Sarkofag
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sarkofag
 */

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace sarkofag {

/*!
 * @brief Sarkofag robot label
 * @ingroup sarkofag
 */
const robot_name_t ROBOT_NAME = "ROBOT_SARKOFAG";

/*!
 * @brief configuration file EDP Sarkofag section string
 * @ingroup sarkofag
 */
const std::string EDP_SECTION = "[edp_sarkofag]";

/*!
 * @brief configuration file ECP Sarkofag section string
 * @ingroup sarkofag
 */
const std::string ECP_SECTION = "[ecp_sarkofag]";

/*!
 * @brief Sarkofag total number of servos
 * @ingroup sarkofag
 */
const int NUM_OF_SERVOS = 1;

/*!
 * @brief Sarkofag first Moxa port number [0..7]
 * @ingroup sarkofag
 */
const int FIRST_MOXA_PORT_NUM = 0;

/*!
 * @brief Sarkofag last Moxa port number [0..7]
 * @ingroup sarkofag
 */
const int LAST_MOXA_PORT_NUM = 0;

/*!
 * @brief Sarkofag overcurrent threshold [mA]
 * @ingroup sarkofag
 */
const int MAX_CURRENT_0 = 20000;

}
} // namespace lib
} // namespace mrrocpp

#endif /* _SARKOFAG_CONST_H */
