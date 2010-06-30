#if !defined(_SPKM_CONST_H)
#define _SPKM_CONST_H

#include "lib/data_port_headers/epos.h"

#define SPKM_NUM_OF_SERVOS	7

namespace mrrocpp {

}

using namespace mrrocpp;

namespace mrrocpp {
namespace lib {
const robot_name_t ROBOT_SPKM = "ROBOT_SPKM";
enum SPKM_CBUFFER_VARIANT
{
	SPKM_CBUFFER_EPOS_LOW_LEVEL_COMMAND, SPKM_CBUFFER_EPOS_GEN_PARAMETERS
};

struct spkm_cbuffer
{
	SPKM_CBUFFER_VARIANT variant;
	union
	{
		epos_low_level_command epos_low_level_command_structure;
		epos_gen_parameters epos_gen_parameters_structure;
	};

};

struct spkm_rbuffer
{
	single_controller_epos_reply epos_controller[SPKM_NUM_OF_SERVOS];
	bool contact;
}__attribute__((__packed__));

#define EDP_SPKM_SECTION "[edp_spkm]"
#define ECP_SPKM_SECTION "[ecp_spkm]"

} // namespace lib
} // namespace mrrocpp

#endif /* _SPKM_CONST_H */
