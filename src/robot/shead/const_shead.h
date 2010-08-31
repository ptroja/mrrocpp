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

const robot_name_t ROBOT_SHEAD = "ROBOT_SHEAD";

enum CBUFFER_VARIANT
{
	CBUFFER_HEAD_SOLIDIFICATION, CBUFFER_VACUUM_ACTIVATION
};

struct cbuffer
{
	CBUFFER_VARIANT variant;
	union
	{
		lib::shead::HEAD_SOLIDIFICATION head_solidification;
		lib::shead::VACUUM_ACTIVATION vacuum_activation;
	};
};

struct rbuffer
{
	reply shead_reply;
};

const std::string EDP_SECTION ="[edp_shead]";
const std::string ECP_SECTION ="[ecp_shead]";

const int SHEAD_NUM_OF_SERVOS = 1;

} // namespace shead
} // namespace lib
} // namespace mrrocpp

#endif /* _SHEAD_CONST_H */
