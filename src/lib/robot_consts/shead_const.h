#if !defined(_SHEAD_CONST_H)
#define _SHEAD_CONST_H

#include "lib/data_port_headers/shead.h"

namespace mrrocpp {

}

using namespace mrrocpp;

namespace mrrocpp {
namespace lib {

enum SHEAD_CBUFFER_VARIANT {
	SHEAD_CBUFFER_HEAD_SOLIDIFICATION, SHEAD_CBUFFER_VACUUM_ACTIVATION
};

struct shead_cbuffer {
	SHEAD_CBUFFER_VARIANT variant;
	union {
		lib::SHEAD_HEAD_SOLIDIFICATION head_solidification;
		lib::SHEAD_VACUUM_ACTIVATION vacuum_activation;
	};

};

struct shead_rbuffer {
	shead_reply reply;
};

#define EDP_SHEAD_SECTION "[edp_shead]"
#define ECP_SHEAD_SECTION "[ecp_shead]"

#define SHEAD_NUM_OF_SERVOS	1

} // namespace lib
} // namespace mrrocpp

#endif /* _SHEAD_CONST_H */
