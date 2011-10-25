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

/*!
 * @brief Condition that finishes after desired time passes
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup mp
 */
class delay_ms_condition : public generator
{
protected:
	/**
	 * @brief timer to measure time
	 */
	lib::timer local_timer;

	/**
	 * @brief desired total time to pass
	 */
	int ms_delay;

public:

	/**
	 * @brief Constructor
	 * @param _mp_task mp task object reference.
	 * @param _ms_delay desired total time to pass
	 */
	delay_ms_condition(task::task& _mp_task, int _ms_delay);

	/**
	 * @brief sets desired total time to pass
	 * @param _ms_delay desired total time to pass
	 */
	void configure(int _ms_delay);

	bool first_step();
	bool next_step();
};

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif /*MP_DELAY_MS_CONDITION_H_*/
