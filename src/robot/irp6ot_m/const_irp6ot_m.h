#if !defined(_IRP6OT_M_CONST_H)
#define _IRP6OT_M_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for IRp6 on track manipulator manipulator
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6ot_m
 */

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace irp6ot_m {

/*!
 * @brief IRp6 on track manipulator robot label
 * @ingroup irp6ot_m
 */
const robot_name_t ROBOT_NAME = "ROBOT_IRP6OT_M";

/*!
 * @brief configuration file EDP IRp6 on track manipulator section string
 * @ingroup irp6ot_m
 */
const std::string EDP_SECTION = "[edp_irp6ot_m]";

/*!
 * @brief configuration file ECP IRp6 on track manipulator section string
 * @ingroup irp6ot_m
 */
const std::string ECP_SECTION = "[ecp_irp6ot_m]";

/*!
 * @brief IRp6 on track manipulator total number of servos
 * @ingroup irp6ot_m
 */
const int NUM_OF_SERVOS = 7;

}
} // namespace lib
} // namespace mrrocpp

#endif /* _IRP6OT_M_CONST_H */
