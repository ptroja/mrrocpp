#if !defined(_SPKM_CONST_H)
#define _SPKM_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "robot/epos/dp_epos.h"

#include "base/lib/impconst.h"
#define SPKM_NUM_OF_SERVOS	7

namespace mrrocpp {
namespace lib {
namespace spkm {

const robot_name_t ROBOT_SPKM = "ROBOT_SPKM";

enum CBUFFER_VARIANT
{
	CBUFFER_EPOS_CUBIC_COMMAND,
	CBUFFER_EPOS_TRAPEZOIDAL_COMMAND,
	CBUFFER_EPOS_OPERATIONAL_COMMAND,
	CBUFFER_EPOS_BRAKE_COMMAND
};

struct cbuffer
{
	CBUFFER_VARIANT variant;
	union
	{
		epos::epos_cubic_command epos_cubic_command_structure;
		epos::epos_trapezoidal_command epos_trapezoidal_command_structure;
		epos::epos_operational_command epos_operational_command_structure;
	};

};

struct rbuffer
{
	epos::single_controller_epos_reply epos_controller[SPKM_NUM_OF_SERVOS];
	bool contact;
}__attribute__((__packed__));

#define EDP_SPKM_SECTION "[edp_spkm]"
#define ECP_SPKM_SECTION "[ecp_spkm]"

} // namespace spkm
} // namespace lib
} // namespace mrrocpp

#endif /* _SPKM_CONST_H */
