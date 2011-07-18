#if !defined(__SBENCH_DATA_PORT_H)
#define __SBENCH_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include <string>

namespace mrrocpp {
namespace lib {
namespace sbench {

/*!
 * @brief SwarmItFix Head head soldification command data port
 * @ingroup sbench
 */
const std::string HEAD_SOLIDIFICATION_DATA_PORT = "SBENCH_HEAD_SOLIDIFICATION_DATA_PORT";

/*!
 * @brief SwarmItFix Head head vacuum activation command data port
 * @ingroup sbench
 */
const std::string VACUUM_ACTIVATION_DATA_PORT = "SBENCH_VACUUM_ACTIVATION_DATA_PORT";

/*!
 * @brief SwarmItFix Head status data request port
 * @ingroup sbench
 */
const std::string REPLY_DATA_REQUEST_PORT = "SBENCH_REPLY_DATA_REQUEST_PORT";

/*!
 * @brief SwarmItFix Head EDP state of the head soldification enum
 * @ingroup sbench
 */
enum STATE_OF_THE_HEAD
{
	HEAD_STATE_SOLDIFIED, HEAD_STATE_DESOLDIFIED, HEAD_STATE_INTERMEDIATE
};

/*!
 * @brief SwarmItFix Head EDP state of the vacuum enum
 * @ingroup sbench
 */
enum STATE_OF_THE_VACUUM
{
	VACUUM_STATE_ON, VACUUM_STATE_OFF, VACUUM_STATE_INTERMEDIATE
};

/*!
 * @brief SwarmItFix Head EDP head soldification command enum
 * @ingroup sbench
 */
enum HEAD_SOLIDIFICATION
{
	SOLIDIFY, DESOLIDIFY, HEAD_SOLIDIFICATION_NO_ACTION
}; // namespace mrrocpp

/*!
 * @brief SwarmItFix Head EDP vacuum activation command enum
 * @ingroup sbench
 */
enum VACUUM_ACTIVATION
{
	VACUUM_ON, VACUUM_OFF, VACUUM_ACTIVATION_NO_ACTION
}; // namespace mrrocpp

/*!
 * @brief SwarmItFix Head reply buffer
 * @ingroup sbench
 */
struct reply
{
	STATE_OF_THE_HEAD head_state;
	STATE_OF_THE_VACUUM vacuum_state;
}__attribute__((__packed__));

} // namespace sbench
}
}

#endif
