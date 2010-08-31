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

const double INC_PER_REVOLUTION = 4000; // Liczba impulsow enkodera na obrot walu - musi byc float
const robot_name_t ROBOT_NAME = "ROBOT_CONVEYOR";

const std::string EDP_SECTION = "[edp_conveyor]";
const std::string ECP_SECTION = "[ecp_conveyor]";

const int NUM_OF_SERVOS = 1;

} // namespace conveyor
} // namespace lib
} // namespace mrrocpp

#endif /* _CONVEYOR_CONST_H */
