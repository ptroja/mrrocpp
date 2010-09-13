#if !defined(_SHEAD_CONST_H)
#define _SHEAD_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include "dp_shead.h"

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace shead {

/*!
 * @brief SwarmItFix Head robot label
 * @ingroup shead
 */
const robot_name_t ROBOT_NAME = "ROBOT_SHEAD";

/*!
 * @brief SwarmItFix Head EDP command buffer variant enum
 * @ingroup shead
 */
enum CBUFFER_VARIANT
{
	CBUFFER_HEAD_SOLIDIFICATION, CBUFFER_VACUUM_ACTIVATION
};

/*!
 * @brief SwarmItFix Head EDP command buffer
 * @ingroup shead
 */
struct cbuffer
{
	CBUFFER_VARIANT variant;
	union
	{
		lib::shead::HEAD_SOLIDIFICATION head_solidification;
		lib::shead::VACUUM_ACTIVATION vacuum_activation;
	};
};

/*!
 * @brief SwarmItFix Head EDP reply buffer
 * @ingroup shead
 */
struct rbuffer
{
	reply shead_reply;
};

/*!
 * @brief configuration file EDP SwarmItFix Head section string
 * @ingroup shead
 */
const std::string EDP_SECTION = "[edp_shead]";

/*!
 * @brief configuration file ECP SwarmItFix Head section string
 * @ingroup shead
 */
const std::string ECP_SECTION = "[ecp_shead]";

/*!
 * @brief SwarmItFix Head total number of servos
 * @ingroup shead
 */
const int NUM_OF_SERVOS = 1;

} // namespace shead
} // namespace lib
} // namespace mrrocpp

#endif /* _SHEAD_CONST_H */
