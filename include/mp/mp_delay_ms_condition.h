#ifndef MP_DELAY_MS_CONDITION_H_
#define MP_DELAY_MS_CONDITION_H_

#include "mp/mp.h"

// condition to wait for desired time in ms

class mp_delay_ms_condition: public mp_generator
{
protected:
	mp_timer* local_timer;
	float sec;
	int ms_delay;

public:

    // konstruktor
    mp_delay_ms_condition(mp_task& _mp_task, int _ms_delay);
    ~mp_delay_ms_condition();

	void configure (int _ms_delay);

	virtual bool first_step ();
	virtual bool next_step ();

};

#endif /*MP_DELAY_MS_CONDITION_H_*/
