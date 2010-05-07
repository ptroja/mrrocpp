#if !defined(_BIRD_HAND_CONST_H)
#define _BIRD_HAND_CONST_H

#include "lib/data_port_headers/epos.h"

namespace mrrocpp {

}

using namespace mrrocpp;

namespace mrrocpp {
namespace lib {

enum BIRD_HAND_CBUFFER_VARIANT {
	BIRD_HAND_CBUFFER_EPOS_LOW_LEVEL_COMMAND,
	BIRD_HAND_CBUFFER_EPOS_GEN_PARAMETERS
};

struct bird_hand_cbuffer {
	BIRD_HAND_CBUFFER_VARIANT variant;
	union {
		epos_low_level_command epos_low_level_command_structure;
		epos_gen_parameters epos_gen_parameters_structure;
	};

};

struct bird_hand_rbuffer {
	bool contact;
	single_controller_epos_reply epos_controller[6];
}__attribute__((__packed__));

#define EDP_BIRD_HAND_SECTION "[edp_bird_hand]"
#define ECP_BIRD_HAND_SECTION "[ecp_bird_hand]"

#define BIRD_HAND_NUM_OF_SERVOS	6

} // namespace lib
} // namespace mrrocpp

#endif /* _BIRD_HAND_CONST_H */
