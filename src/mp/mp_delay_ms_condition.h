#ifndef MP_DELAY_MS_CONDITION_H_
#define MP_DELAY_MS_CONDITION_H_

#include "mp/mp.h"

namespace mrrocpp {
namespace mp {
namespace generator {

// condition to wait for desired time in ms

class delay_ms_condition: public generator
{
protected:
	lib::timer* local_timer;
	float sec;
	int ms_delay;

public:

    // konstruktor
    delay_ms_condition(task::task& _mp_task, int _ms_delay);
    ~delay_ms_condition();

	void configure (int _ms_delay);

	virtual bool first_step ();
	virtual bool next_step ();

};

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif /*MP_DELAY_MS_CONDITION_H_*/
