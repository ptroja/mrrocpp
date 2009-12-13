// -------------------------------------------------------------------------
//                                   reader.h
// -------------------------------------------------------------------------

#ifndef __EDP_EXTENSION_THREAD_H
#define __EDP_EXTENSION_THREAD_H

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

class effector;

class edp_extension_thread
{
private:
	effector &master;

public:
    pthread_t thread_id;
//	static void *thread_start(void* arg);
    virtual void *thread_main_loop(void* arg) = 0;


    edp_extension_thread(effector &_master);
    ~edp_extension_thread();
};


} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
