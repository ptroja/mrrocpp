#if !defined(_CONVEYOR_CONST_H)
#define _CONVEYOR_CONST_H

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {

#define CONVEYOR_INC_PER_REVOLUTION  4000.0  // Liczba impulsow enkodera na obrot walu - musi byc float
const robot_name_t ROBOT_CONVEYOR = "ROBOT_CONVEYOR";

#define EDP_CONVEYOR_SECTION "[edp_conveyor]"
#define ECP_CONVEYOR_SECTION "[ecp_conveyor]"

#define CONVEYOR_NUM_OF_SERVOS		1

} // namespace lib
} // namespace mrrocpp

#endif /* _CONVEYOR_CONST_H */
