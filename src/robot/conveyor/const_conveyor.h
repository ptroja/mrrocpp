#if !defined(_CONVEYOR_CONST_H)
#define _CONVEYOR_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for Conveyor
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup conveyor
 */

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace conveyor {

/*!
 * @brief Conveyor robot label
 * @ingroup conveyor
 */
const robot_name_t ROBOT_NAME = "ROBOT_CONVEYOR";

/*!
 * @brief configuration file EDP Conveyor section string
 * @ingroup conveyor
 */
static const std::string EDP_SECTION = "[edp_conveyor]";

/*!
 * @brief configuration file ECP Conveyor section string
 * @ingroup conveyor
 */
const std::string ECP_SECTION = "[ecp_conveyor]";

/*!
 * @brief Conveyor total number of servos
 * @ingroup conveyor
 */
const int NUM_OF_SERVOS = 1;

} // namespace conveyor
} // namespace lib
} // namespace mrrocpp

#endif /* _CONVEYOR_CONST_H */
