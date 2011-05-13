#if !defined(_POLYCRANK_CONST_H)
#define _POLYCRANK_CONST_H

#include "base/lib/impconst.h"
#include <string>

namespace mrrocpp {
namespace lib {
namespace polycrank {

const int NUM_OF_SERVOS = 7;
const int LAST_MOXA_PORT_NUM = 0;

/*!
 * @brief Bird Hand robot label lower case
 * @ingroup polycrank
 */
const robot_name_t ROBOT_NAME = "polycrank";

const std::string ports_strings[] =
		{ "/dev/ttyMI0", "/dev/ttyMI1", "/dev/ttyMI2", "/dev/ttyMI3", "/dev/ttyMI4", "/dev/ttyMI5", "/dev/ttyMI6" };

}
} // namespace lib
} // namespace mrrocpp

#endif /* _POLYCRANK_CONST_H */
