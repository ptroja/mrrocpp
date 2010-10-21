#if !defined(__SHEAD_DATA_PORT_H)
#define __SHEAD_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include <string>

namespace mrrocpp {
namespace lib {
namespace shead {

/*!
 * @brief SwarmItFix Head head soldification command data port
 * @ingroup shead
 */
const std::string HEAD_SOLIDIFICATION_DATA_PORT = "SHEAD_HEAD_SOLIDIFICATION_DATA_PORT";

/*!
 * @brief SwarmItFix Head head vacuum activation command data port
 * @ingroup shead
 */
const std::string VACUUM_ACTIVATION_DATA_PORT = "SHEAD_VACUUM_ACTIVATION_DATA_PORT";

/*!
 * @brief SwarmItFix Head status data request port
 * @ingroup shead
 */
const std::string REPLY_DATA_REQUEST_PORT = "SHEAD_REPLY_DATA_REQUEST_PORT";

/*!
 * @brief SwarmItFix Head EDP state of the head soldification enum
 * @ingroup shead
 */
enum STATE_OF_THE_HEAD
{
	HEAD_STATE_SOLDIFIED, HEAD_STATE_DESOLDIFIED, HEAD_STATE_INTERMEDIATE
};

/*!
 * @brief SwarmItFix Head EDP state of the vacuum enum
 * @ingroup shead
 */
enum STATE_OF_THE_VACUUM
{
	VACUUM_STATE_ON, VACUUM_STATE_OFF, VACUUM_STATE_INTERMEDIATE
};

/*!
 * @brief SwarmItFix Head EDP head soldification command enum
 * @ingroup shead
 */
enum HEAD_SOLIDIFICATION
{
	SOLIDIFY, DESOLIDIFY, HEAD_SOLIDIFICATION_NO_ACTION
}; // namespace mrrocpp

/*!
 * @brief SwarmItFix Head EDP vacuum activation command enum
 * @ingroup shead
 */
enum VACUUM_ACTIVATION
{
	VACUUM_ON, VACUUM_OFF, VACUUM_ACTIVATION_NO_ACTION
}; // namespace mrrocpp

/*!
 * @brief SwarmItFix Head reply buffer
 * @ingroup shead
 */
struct reply
{
	STATE_OF_THE_HEAD head_state;
	STATE_OF_THE_VACUUM vacuum_state;
};

} // namespace shead
}
}

#endif
