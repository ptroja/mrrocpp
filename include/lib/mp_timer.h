#ifndef MP_TIMER_H
#define MP_TIMER_H

#include <stdint.h>

   enum mp_timer_status_enum {
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


/**************************** mp_timer *****************************/
class mp_timer {

private:

	uint64_t cycles;
	uint64_t ncycles;
	uint64_t t1, t2;
	int timer_initialized;
	int timer_started;
	int timer_stopped;
	mp_timer_status_enum last_status;

public:

	// konstruktor
	mp_timer (void);
	mp_timer_status_enum timer_start(uint64_t *t1);
	mp_timer_status_enum timer_stop(uint64_t *t2);
	mp_timer_status_enum get_time(float *sec);
	mp_timer_status_enum get_cycles(uint64_t *c, uint64_t *nc);
	void print_last_status(void);

}; // end: class edp_buffer
/**************************** mp_timer *****************************/

#endif
