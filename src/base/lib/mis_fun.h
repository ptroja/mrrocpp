/*!
 * @file mis_fun.h
 * @brief Thread utility functions declarations.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#ifndef __MIS_FUN_H
#define __MIS_FUN_H

namespace mrrocpp {
namespace lib {

#include <pthread.h>

//! Set the priority of POSIX thread
void set_thread_priority(pthread_t thread, int sched_priority_l);

//! Set thread name if available (for QNX Momentics debugger)
//! @return status 0 if successful
int set_thread_name(const char *);

} // namespace lib
} // namespace mrrocpp

#endif
