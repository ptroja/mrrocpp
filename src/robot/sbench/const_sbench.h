#if !defined(_SBENCH_CONST_H)
#define _SBENCH_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include "dp_sbench.h"

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace sbench {

/*!
 * @brief SwarmItFix Head robot label
 * @ingroup sbench
 */
const robot_name_t ROBOT_NAME = "sbench";

/*!
 * @brief SwarmItFix Head EDP command buffer variant enum
 * @ingroup sbench
 */
enum CBUFFER_VARIANT
{
	CBUFFER_HEAD_SOLIDIFICATION, CBUFFER_VACUUM_ACTIVATION
};

/*!
 * @brief SwarmItFix Head EDP command buffer
 * @ingroup sbench
 */
struct cbuffer
{
	CBUFFER_VARIANT variant;
	union
	{
		lib::sbench::HEAD_SOLIDIFICATION head_solidification;
		lib::sbench::VACUUM_ACTIVATION vacuum_activation;
	};
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Head EDP reply buffer
 * @ingroup sbench
 */
struct rbuffer
{
	reply sbench_reply;
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Head total number of servos
 * @ingroup sbench
 */
const int NUM_OF_SERVOS = 1;

} // namespace sbench
} // namespace lib
} // namespace mrrocpp

#endif /* _SBENCH_CONST_H */
