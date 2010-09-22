#if !defined(_SPKM_CONST_H)
#define _SPKM_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "robot/spkm/dp_spkm.h"

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace spkm {

/*!
 * @brief SwarmItFix Parallel Kinematic Machine robot label
 * @ingroup spkm
 */
const robot_name_t ROBOT_NAME = "ROBOT_SPKM";

/*!
 * @brief SwarmItFix Parallel Kinematic Machine EDP command buffer variant enum
 * @ingroup spkm
 */
enum CBUFFER_VARIANT
{
	CBUFFER_EPOS_CUBIC_COMMAND,
	CBUFFER_EPOS_TRAPEZOIDAL_COMMAND,
	CBUFFER_EPOS_OPERATIONAL_COMMAND,
	CBUFFER_EPOS_BRAKE_COMMAND
};

/*!
 * @brief SwarmItFix Parallel Kinematic Machine EDP command buffer
 * @ingroup spkm
 */
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

/*!
 * @brief SwarmItFix Parallel Kinematic Machine EDP reply buffer
 * @ingroup spkm
 */
struct rbuffer
{
	epos::single_controller_epos_reply epos_controller[NUM_OF_SERVOS];
	bool contact;
};

/*!
 * @brief configuration file EDP SwarmItFix Parallel Kinematic Machine section string
 * @ingroup spkm
 */
const std::string EDP_SECTION = "[edp_spkm]";

/*!
 * @brief configuration file ECP SwarmItFix Parallel Kinematic Machine section string
 * @ingroup spkm
 */
const std::string ECP_SECTION = "[ecp_spkm]";

} // namespace spkm
} // namespace lib
} // namespace mrrocpp

#endif /* _SPKM_CONST_H */
