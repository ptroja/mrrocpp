// -------------------------------------------------------------------------
//                                   reader.h
// -------------------------------------------------------------------------

#ifndef __EDP_VSP_T_H
#define __EDP_VSP_T_H

#include <stdint.h>
#include <pthread.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

namespace mrrocpp {
namespace edp {
namespace common {

class manip_effector;

class edp_vsp
{
private:
	manip_effector &master;

public:
    void operator()(void);

    edp_vsp(manip_effector &_master);
};



} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
