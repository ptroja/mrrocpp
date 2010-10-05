/*!
 * @file
 * @brief File contains ecp base sub_task definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include <cstring>
#include <unistd.h>
#include <cerrno>
#include <cctype>
#include <cstdio>
#include <boost/foreach.hpp>

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_sub_task.h"
#include "base/ecp/ecp_robot.h"
#include "base/ecp/ECP_main_error.h"
#include "base/ecp/ecp_generator.h"

#if defined(USE_MESSIP_SRR)
#include "base/lib/messip/messip_dataport.h"
#endif

namespace mrrocpp {
namespace ecp {
namespace common {
namespace sub_task {

sub_task::sub_task(task::task &_ecp_t) :
	ecp_t(_ecp_t), sr_ecp_msg(*(_ecp_t.sr_ecp_msg))
{
}

} // namespace sub_task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
