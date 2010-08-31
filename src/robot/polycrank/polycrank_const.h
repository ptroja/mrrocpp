#if !defined(_POLYCRANK_CONST_H)
#define _POLYCRANK_CONST_H

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace polycrank {

const robot_name_t ROBOT_POLYCRANK = "ROBOT_POLYCRANK";

const std::string EDP_POLYCRANK_SECTION = "[edp_polycrank]";
const std::string ECP_POLYCRANK_SECTION = "[ecp_polycrank]";

#define POLYCRANK_NUM_OF_SERVOS	8

}
} // namespace lib
} // namespace mrrocpp

#endif /* _POLYCRANK_CONST_H */
