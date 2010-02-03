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
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

#include "lib/impconst.h"

namespace mrrocpp {
namespace lib {


class boost_condition_synchroniser {
public:
	// wait for condition and new_command;
	void wait();
	// set new_command;
	void command();
	// set has_command=0;
	void null_command();
	boost_condition_synchroniser();

private:
	boost::condition_variable cond; //! active command condition
	boost::mutex mtx; //! mutex related to condition variable

	bool has_command; //! flag indicating active command to execute

};

// setting of thread priority
void set_thread_priority(pthread_t thread, int sched_priority_l);

// set thread name (for QNX Momentics debugger)
int set_thread_name(const char *);

} // namespace lib
} // namespace mrrocpp

#endif
