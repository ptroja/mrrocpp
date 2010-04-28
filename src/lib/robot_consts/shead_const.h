#if !defined(_SHEAD_CONST_H)
#define _SHEAD_CONST_H

#include "lib/swarmitfix.h"

namespace mrrocpp {

}

using namespace mrrocpp;

namespace mrrocpp {
namespace lib {

struct shead_cbuffer {
	lib::SHEAD_HEAD_SOLIDIFICATION head_solidification;
	lib::SHEAD_VACUUM_ACTIVATION vacuum_activation;
};

struct shead_rbuffer {
	bool head_solfified;
	bool vacuum_on;
};

#define EDP_SHEAD_SECTION "[edp_shead]"
#define ECP_SHEAD_SECTION "[ecp_shead]"

#define SHEAD_NUM_OF_SERVOS	1

} // namespace lib
} // namespace mrrocpp

#endif /* _SHEAD_CONST_H */
