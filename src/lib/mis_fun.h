// ------------------------------------------------------------------------
// Plik:				mis_fun.h
// Opis:			miscellaneous functions
// ------------------------------------------------------------------------

#ifndef __MIS_FUN_H
#define __MIS_FUN_H

#include "lib/condition_synchroniser.h"

namespace mrrocpp {
namespace lib {

#include <pthread.h>

// setting of thread priority
void set_thread_priority(pthread_t thread, int sched_priority_l);

// set thread name (for QNX Momentics debugger)
int set_thread_name(const char *);

} // namespace lib
} // namespace mrrocpp

#endif
