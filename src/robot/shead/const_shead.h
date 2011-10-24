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
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Head EDP reply buffer
 * @ingroup shead
 */
struct rbuffer
{
	reply shead_reply;
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Head total number of servos
 * @ingroup shead
 */
const int NUM_OF_SERVOS = 1;

} // namespace shead
} // namespace lib
} // namespace mrrocpp

#endif /* _SHEAD_CONST_H */
