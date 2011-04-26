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
#include <time.h>

//! Set the process scheduler
void set_process_sched();

//! Set the priority of POSIX thread
void set_thread_priority(pthread_t thread, int sched_priority_l);

//! Set thread name if available (for QNX Momentics debugger)
//! @return status 0 if successful
int set_thread_name(const char *);

//! Increment timespec structure with a number of nanoseconds taking care about overflows
//! @param ts pointer to the timespec structure
//! @param increment value of the increment in nanoseconds
//! @note This is a C-style call so we want a pointer instead of a C++ reference
void timespec_increment_ns(struct timespec * ts, unsigned long increment);

} // namespace lib
} // namespace mrrocpp

#endif
