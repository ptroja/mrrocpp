#ifndef TIMER_H
#define TIMER_H

#include <time.h>

namespace mrrocpp {
namespace lib {

enum timer_status_enum {
	TIMER_INITIALIZED,
	TIMER_STARTED,
	TIMER_STOPPED,
	TIME_RETRIVED,
	CYCLES_RETRIVED,
	TIMER_NOT_INITIALIZED,
	TIMER_NOT_STARTED,
	TIMER_NOT_STOPPED,
	TIMER_RUNNING_OR_NOT_STARTED
};


/**************************** timer *****************************/
class timer {

private:

	struct timespec t1, t2;
	bool timer_initialized;
	bool timer_started;
	bool timer_stopped;
	timer_status_enum last_status;

public:

	// konstruktor
	timer (void);
	timer_status_enum timer_start(void);
	timer_status_enum timer_stop(void);
	timer_status_enum get_time(float *sec);
	void print_last_status(void);

}; // end: class edp_buffer
/**************************** timer *****************************/

} // namespace lib
} // namespace mrrocpp

#endif
