// -------------------------------------------------------------------------
//                                   reader.h
// -------------------------------------------------------------------------

#ifndef __EDP_VSP_T_H
#define __EDP_VSP_T_H

#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "kinematics/common/transformer_error.h"


namespace mrrocpp {
namespace edp {
namespace common {

class manip_and_conv_effector;

class vis_server
{
private:
	manip_and_conv_effector &master;

public:
	static void *thread_start(void* arg);
    void *thread_main_loop(void* arg);


    vis_server(manip_and_conv_effector &_master);
    ~vis_server();
};




} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
