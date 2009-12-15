#ifndef ECP_MP_ROBOT_H_
#define ECP_MP_ROBOT_H_

#include <map>
#include "lib/srlib.h"
#include "lib/sensor.h"
#include "ecp_mp/transmitter/transmitter.h"

namespace mrrocpp {
namespace ecp_mp {

class robot
{
public:
	const lib::robot_name_t robot_name; // by Y - nazwa robota (track, postument etc.)

	robot(lib::robot_name_t _robot_name);
};

} // namespace ecp_mp
} // namespace mrrocpp


#endif
