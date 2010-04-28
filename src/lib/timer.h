#ifndef TIMER_H
#define TIMER_H

#include <time.h>

namespace mrrocpp {
namespace lib {

class timer {
public:
	typedef enum _timer_status_enum {
		TIMER_INITIALIZED,
		TIMER_STARTED,
		TIMER_STOPPED,
		TIME_RETRIVED,
		CYCLES_RETRIVED,
		TIMER_NOT_INITIALIZED,
		TIMER_NOT_STARTED,
		TIMER_NOT_STOPPED,
		TIMER_RUNNING_OR_NOT_STARTED
	} timer_status_t;

private:
	struct timespec t1, t2;
	bool timer_initialized;
	bool timer_started;
	bool timer_stopped;
	timer_status_t last_status;

public:
	// konstruktor
	timer (void);
	timer_status_t timer_start(void);
	timer_status_t timer_stop(void);
	timer_status_t get_time(float *sec);
	void print_last_status(void);

};

} // namespace lib
} // namespace mrrocpp

#endif
