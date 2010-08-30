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

const robot_name_t ROBOT_SHEAD = "ROBOT_SHEAD";

enum SHEAD_CBUFFER_VARIANT
{
	SHEAD_CBUFFER_HEAD_SOLIDIFICATION, SHEAD_CBUFFER_VACUUM_ACTIVATION
};

struct shead_cbuffer
{
	SHEAD_CBUFFER_VARIANT variant;
	union
	{
		lib::SHEAD_HEAD_SOLIDIFICATION head_solidification;
		lib::SHEAD_VACUUM_ACTIVATION vacuum_activation;
	};
};

struct shead_rbuffer
{
	shead_reply reply;
};

#define EDP_SHEAD_SECTION "[edp_shead]"
#define ECP_SHEAD_SECTION "[ecp_shead]"

#define SHEAD_NUM_OF_SERVOS	1

} // namespace lib
} // namespace mrrocpp

#endif /* _SHEAD_CONST_H */
