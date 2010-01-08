// ------------------------------------------------------------------------
// Plik:				mis_fun.h
// System:		QNX/MRROC++   v. 6.3
// Opis:			miscelangeous functions
// Modyfikacja:
// Jej autor:
// Data:			2006
// ------------------------------------------------------------------------

#ifndef __MIS_FUN_H
#define __MIS_FUN_H

#include <pthread.h>
#include <string.h>

#include "lib/impconst.h"

namespace mrrocpp {
namespace lib {

// setting of thread priority
void set_thread_priority(pthread_t thread, int sched_priority_l);

// set thread name (for QNX Momentics debugger)
int set_thread_name(const char *);

} // namespace lib
} // namespace mrrocpp

#endif
