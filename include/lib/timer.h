#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

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

	uint64_t cycles;
	uint64_t ncycles;
	uint64_t t1, t2;
	int timer_initialized;
	int timer_started;
	int timer_stopped;
	timer_status_enum last_status;

public:

	// konstruktor
	timer (void);
	timer_status_enum timer_start(uint64_t *t1);
	timer_status_enum timer_stop(uint64_t *t2);
	timer_status_enum get_time(float *sec);
	timer_status_enum get_cycles(uint64_t *c, uint64_t *nc);
	void print_last_status(void);

}; // end: class edp_buffer
/**************************** timer *****************************/

#endif
