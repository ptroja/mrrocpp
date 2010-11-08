#if !defined(_POLYCRANK_CONST_H)
#define _POLYCRANK_CONST_H

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace polycrank {

const robot_name_t ROBOT_NAME = "ROBOT_POLYCRANK";

const std::string EDP_SECTION = "[edp_polycrank]";
const std::string ECP_SECTION = "[ecp_polycrank]";

const int NUM_OF_SERVOS = 8;

}
} // namespace lib
} // namespace mrrocpp

#endif /* _POLYCRANK_CONST_H */
