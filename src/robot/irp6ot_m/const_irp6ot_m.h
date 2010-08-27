#if !defined(_IRP6OT_M_CONST_H)
#define _IRP6OT_M_CONST_H

/*!
 * @file const_irp6ot_m.h
 * @brief File contains constants and structures for IRp6 on track manipulator
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6ot_m
 */

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {

const robot_name_t ROBOT_IRP6OT_M = "ROBOT_IRP6OT_M";

#define IRP6_ON_TRACK_AXIS_0_TO_5_INC_PER_REVOLUTION   682.0  // Liczba impulsow rezolwera na obrot walu - musi byc float
#define IRP6_ON_TRACK_AXIS_6_INC_PER_REVOLUTION  2000.0  // Liczba impulsow enkodera na obrot walu - musi byc float
#define EDP_IRP6OT_M_SECTION "[edp_irp6ot_m]"
#define ECP_IRP6OT_M_SECTION "[ecp_irp6ot_m]"

#define IRP6OT_M_NUM_OF_SERVOS	7

} // namespace lib
} // namespace mrrocpp

#endif /* _IRP6OT_M_CONST_H */
