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

const robot_name_t ROBOT_SARKOFAG = "ROBOT_SARKOFAG";

#define SARKOFAG_INC_PER_REVOLUTION  4000.0  // Liczba impulsow enkodera na obrot walu - musi byc float
const std::string EDP_SARKOFAG_SECTION ="[edp_sarkofag]";
const std::string ECP_SARKOFAG_SECTION ="[ecp_sarkofag]";

#define SARKOFAG_NUM_OF_SERVOS	1

}
} // namespace lib
} // namespace mrrocpp

#endif /* _SARKOFAG_CONST_H */
