#if !defined(_IRP6M_CONST_H)
#define _IRP6M_CONST_H

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace irp6m {

const robot_name_t ROBOT_NAME = "ROBOT_IRP6M";
const std::string EDP_SECTION = "[edp_irp6m]";
const std::string ECP_SECTION = "[ecp_irp6m]";

const int NUM_OF_SERVOS = 5;

}
} // namespace lib
} // namespace mrrocpp

#endif /* _IRP6M_CONST_H */
