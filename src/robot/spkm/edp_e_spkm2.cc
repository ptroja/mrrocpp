#include <iostream>
#include <fstream>
#include <cstdio>
#include <sys/stat.h>
//<sys/types.h>
#include <boost/foreach.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

#include "robot/spkm/edp_e_spkm2.h"
#include "base/edp/reader.h"
//#include "base/edp/vis_server.h"

#include "robot/spkm/kinematic_model_spkm.h"
#include "robot/spkm/kinematic_parameters_spkm.h"
#include "base/edp/manip_trans_t.h"

#include "robot/epos/epos.h"
#include "robot/epos/epos_access_usb.h"
#include "base/lib/pvt.hpp"
#include "base/lib/pvat_cartesian.hpp"

#include "robot/spkm/exceptions.h"
#include "robot/epos/epos_exceptions.hpp"

#include "robot/epos/ipm_executor.h"

namespace mrrocpp {
namespace edp {
namespace spkm2 {

effector::effector(common::shell &_shell) :
	spkm::effector(_shell)
{
}

} // namespace spkm2


namespace common {

// Create spkm effector.
effector* return_created_efector(common::shell &_shell)
{
	return new spkm2::effector(_shell);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
