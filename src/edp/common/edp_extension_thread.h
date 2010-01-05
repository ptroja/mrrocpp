// -------------------------------------------------------------------------
//                                   reader.h
// -------------------------------------------------------------------------

#ifndef __EDP_EXTENSION_THREAD_H
#define __EDP_EXTENSION_THREAD_H

#include <pthread.h>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

namespace mrrocpp {
namespace edp {
namespace common {

class effector;

class edp_extension_thread
{
private:
	effector &master;

protected:
    pthread_t thread_id;

    virtual void *thread_main_loop(void* arg) = 0;

    edp_extension_thread(effector &_master);
    virtual ~edp_extension_thread();
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
