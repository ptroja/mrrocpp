#if !defined(_POLYCRANK_CONST_H)
#define _POLYCRANK_CONST_H

#include "base/lib/impconst.h"
#include <string>

namespace mrrocpp {
namespace lib {
namespace polycrank {

const robot_name_t ROBOT_NAME = "ROBOT_POLYCRANK";

const std::string EDP_SECTION = "[edp_polycrank]";
const std::string ECP_SECTION = "[ecp_polycrank]";

const int NUM_OF_SERVOS = 7;
const int LAST_MOXA_PORT_NUM = 0;

/*!
 * @brief IRp6 conveyor array of communication port names
 * @ingroup conveyor
 */
//const std::string ports_strings[] = {"/dev/ser7","/dev/ser3"};


const std::string ports_strings[] =
		{ "/dev/ttyMI0", "/dev/ttyMI1", "/dev/ttyMI2", "/dev/ttyMI3", "/dev/ttyMI4", "/dev/ttyMI5", "/dev/ttyMI6" };


/*!
 * @brief Conveyor robot label
 * @ingroup conveyor
 */
/*
 const robot_name_t ROBOT_NAME = "ROBOT_CONVEYOR";
 static const std::string EDP_SECTION = "[edp_conveyor]";
 const std::string ECP_SECTION = "[ecp_conveyor]";
 const int NUM_OF_SERVOS = 1;
 const int LAST_MOXA_PORT_NUM = 0;
 const std::string ports_strings[] = {"/dev/ser9"};
 */

}
} // namespace lib
} // namespace mrrocpp

#endif /* _POLYCRANK_CONST_H */
