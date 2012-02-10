#if !defined(_IRP6OT_M_CONST_H)
#define _IRP6OT_M_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for IRp6 on track manipulator
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6ot_m
 */

#include<string>

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace irp6ot_m {

/*!
 * @brief IRp6 on track manipulator robot label
 * @ingroup irp6ot_m
 */
const robot_name_t ROBOT_NAME = "irp6ot_m";

/*!
 * @brief IRp6 on track manipulator total number of servos
 * @ingroup irp6ot_m
 */
const int NUM_OF_SERVOS = 7;

/*!
 * @brief IRp6 on track last Moxa port number [0..7]
 * @ingroup irp6ot_m
 */
const int LAST_MOXA_PORT_NUM = 6;

/*!
 * @brief IRp6 on track array of communication port names
 * @ingroup irp6ot_m
 */

const std::string ports_strings[] =
//{ "/dev/ttyMUE7", "/dev/ttyMUE0", "/dev/ttyMUE1", "/dev/ttyMUE2", "/dev/ttyMUE3", "/dev/ttyMUE4", "/dev/ttyMUE5" };
 {"/dev/ttyMI7", "/dev/ttyMI0", "/dev/ttyMI1", "/dev/ttyMI2", "/dev/ttyMI3", "/dev/ttyMI4", "/dev/ttyMI5" };
/*!
 * @brief IRp6 on track overcurrent threshold [mA]
 * @ingroup irp6ot_m
 */
const int MAX_CURRENT_0 = 25000;
const int MAX_CURRENT_1 = 15000;
const int MAX_CURRENT_2 = 18000;
const int MAX_CURRENT_3 = 10000;
const int MAX_CURRENT_4 = 10000;
const int MAX_CURRENT_5 = 10000;
const int MAX_CURRENT_6 = 10000;

/*!
 * @brief IRp6 on track max encoder increment
 * @ingroup irp6ot_m
 */
const double MAX_INCREMENT[] = { 1000, 1000, 1000, 1000, 1000, 1000, 1000 };

} // namespace irp6ot_m
} // namespace lib
} // namespace mrrocpp

#endif /* _IRP6OT_M_CONST_H */
