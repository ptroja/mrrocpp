#include <cstdlib>

#include "ping.h"

namespace mrrocpp {
namespace lib {

bool ping(const std::string & target)
{

	std::string cmd;

	cmd = "ping ";
	cmd += " -c 1 -W 1 ";
	cmd += target;
	//cmd += " 1>/dev/null 2>/dev/null";

	return (system(cmd.c_str()) == 0);
}

}
}
