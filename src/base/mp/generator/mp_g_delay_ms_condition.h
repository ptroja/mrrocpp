#ifndef MP_DELAY_MS_CONDITION_H_
#define MP_DELAY_MS_CONDITION_H_

/*!
 * @file
 * @brief File contains mp delay_ms_condition generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include "base/mp/generator/mp_generator.h"
#include "base/lib/timer.h"

namespace mrrocpp {
namespace mp {
namespace generator {

// condition to wait for desired time in ms

class delay_ms_condition : public generator
{
protected:
	lib::timer local_timer;
	float sec;
	int ms_delay;

public:

	// konstruktor
	delay_ms_condition(task::task& _mp_task, int _ms_delay);

	void configure(int _ms_delay);

	virtual bool first_step();
	virtual bool next_step();
};

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif /*MP_DELAY_MS_CONDITION_H_*/
