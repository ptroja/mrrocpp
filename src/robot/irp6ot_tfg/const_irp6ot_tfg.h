#if !defined(_IRP6OT_TFG_CONST_H)
#define _IRP6OT_TFG_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for IRp6 on track two finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6ot_tfg
 */

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace irp6ot_tfg {

const robot_name_t ROBOT_IRP6OT_TFG = "ROBOT_IRP6OT_TFG";

#define IRP6_ON_TRACK_AXIS_7_INC_PER_REVOLUTION  128.0  // Liczba impulsow enkodera na obrot walu - musi byc float
#define EDP_IRP6OT_TFG_SECTION "[edp_irp6ot_tfg]"
#define ECP_IRP6OT_TFG_SECTION "[ecp_irp6ot_tfg]"

#define IRP6OT_TFG_NUM_OF_SERVOS	1

}
} // namespace lib
} // namespace mrrocpp

#endif /* _IRP6OT_TFG_CONST_H */
