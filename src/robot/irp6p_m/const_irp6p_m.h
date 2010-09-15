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

/*!
 * @brief IRp6 postument manipulator robot label
 * @ingroup irp6p_m
 */
const robot_name_t ROBOT_NAME = "ROBOT_IRP6P_M";

/*!
 * @brief configuration file EDP IRp6 postument manipulator section string
 * @ingroup irp6p_m
 */
const std::string EDP_SECTION = "[edp_irp6p_m]";

/*!
 * @brief configuration file ECP IRp6 postument manipulator section string
 * @ingroup irp6p_m
 */
const std::string ECP_SECTION = "[ecp_irp6p_m]";

/*!
 * @brief IRp6 postument manipulator total number of servos
 * @ingroup irp6p_m
 */
const int NUM_OF_SERVOS = 6;

/*!
 * @brief IRp6 postument first Moxa port number [0..7]
 * @ingroup irp6p_m
 */
const int FIRST_MOXA_PORT_NUM = 0;

/*!
 * @brief IRp6 postument last Moxa port number [0..7]
 * @ingroup irp6p_m
 */
const int LAST_MOXA_PORT_NUM = 5;

} // namespace irp6p_m
} // namespace lib
} // namespace mrrocpp

#endif /* _IRP6P_M_CONST_H */
