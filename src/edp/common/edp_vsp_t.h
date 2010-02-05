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

class irp6s_postument_track_effector;

class edp_vsp
{
private:
	irp6s_postument_track_effector &master;

public:
    void operator()(void);

    edp_vsp(irp6s_postument_track_effector &_master);
};



} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
