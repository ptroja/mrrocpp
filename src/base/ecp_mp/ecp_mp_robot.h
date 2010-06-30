#ifndef ECP_MP_ROBOT_H_
#define ECP_MP_ROBOT_H_

#include "lib/impconst.h"

namespace mrrocpp {
namespace ecp_mp {

class robot
{
public:
	// by Y - nazwa robota (track, postument etc.)
	const lib::robot_name_t robot_name;

	robot(lib::robot_name_t _robot_name);
};

} // namespace ecp_mp
} // namespace mrrocpp


#endif
