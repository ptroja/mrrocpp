// -------------------------------------------------------------------------
//                                   reader.h
// -------------------------------------------------------------------------

#ifndef __EDP_EXTENSION_THREAD_H
#define __EDP_EXTENSION_THREAD_H

#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

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

    virtual void *thread_main_loop(void* arg) = 0;

    edp_extension_thread(effector &_master);
    ~edp_extension_thread();
};

class edp_boost_thread
{
private:
	effector &master;

public:
	boost::thread* thread_id;

    virtual void operator()() = 0;
    virtual void create_thread(void) = 0;
    edp_boost_thread(effector &_master);
    ~edp_boost_thread();
};



} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
