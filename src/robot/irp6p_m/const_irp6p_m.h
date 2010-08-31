#if !defined(_IRP6P_M_CONST_H)
#define _IRP6P_M_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for IRp6 postument manipulator
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6p_m
 */

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace irp6p_m {

#define IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION  4000.0  // Liczba impulsow enkodera na obrot walu - musi byc float
#define IRP6_POSTUMENT_AXIS_6_INC_PER_REVOLUTION  2000.0  // Liczba impulsow enkodera na obrot walu - musi byc float
const robot_name_t ROBOT_IRP6P_M = "ROBOT_IRP6P_M";
#define EDP_IRP6P_M_SECTION "[edp_irp6p_m]"
#define ECP_IRP6P_M_SECTION "[ecp_irp6p_m]"

#define IRP6P_M_NUM_OF_SERVOS	6

} // namespace irp6p_m
} // namespace lib
} // namespace mrrocpp

#endif /* _IRP6P_M_CONST_H */
