#if !defined(_SPKM_CONST_H)
#define _SPKM_CONST_H

#include "lib/swarmitfix.h"
#include "lib/data_port_headers/epos.h"

namespace mrrocpp {

}

using namespace mrrocpp;

namespace mrrocpp {
namespace lib {


enum SPKM_CBUFFER_VARIANT {
	SPKM_CBUFFER_EPOS_LOW_LEVEL_COMMAND, SPKM_CBUFFER_EPOS_GEN_PARAMETERS, SPKM_CBUFFER_NO_ACTION
};

struct spkm_cbuffer {
	SPKM_CBUFFER_VARIANT variant;
	union {
		epos_low_level_command epos_data_port_command_structure;
		epos_gen_parameters epos_data_port_gen_parameters_structure;
	};

};

struct spkm_rbuffer {
	double position[6];
	bool motion_in_progress[6];
	bool contact;
};

#define EDP_SPKM_SECTION "[edp_spkm]"
#define ECP_SPKM_SECTION "[ecp_spkm]"

#define SPKM_NUM_OF_SERVOS	6

} // namespace lib
} // namespace mrrocpp

#endif /* _SPKM_CONST_H */
