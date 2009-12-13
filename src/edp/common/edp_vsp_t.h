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

class irp6s_postument_track_effector;

class edp_vsp
{
private:
	irp6s_postument_track_effector &master;

public:
    pthread_t thread_id;
	static void *thread_start(void* arg);
    void *thread_main_loop(void* arg);


    edp_vsp(irp6s_postument_track_effector &_master);
    ~edp_vsp();
};




} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
