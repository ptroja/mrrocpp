// -------------------------------------------------------------------------
//                                   reader.h
// -------------------------------------------------------------------------

#ifndef __VIS_SERVER_H
#define __VIS_SERVER_H

#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "kinematics/common/transformer_error.h"
#include "edp/common/edp_extension_thread.h"

namespace mrrocpp {
namespace edp {
namespace common {

// TODO: remove forward declarations
class manip_and_conv_effector;

class vis_server : public edp_boost_thread
{
private:
	manip_and_conv_effector &master;

public:
	//! main loop
	void operator()();
	void create_thread(void);

    vis_server(manip_and_conv_effector &_master);
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
