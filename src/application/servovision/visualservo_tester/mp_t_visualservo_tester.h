/*
 * mp_t_visualservo_tester.h
 *
 *  Created on: May 28, 2010
 *      Author: mboryn
 */

#ifndef MP_T_VISUALSERVO_TESTER_H_
#define MP_T_VISUALSERVO_TESTER_H_

#include "base/mp/MP_main_error.h"

#include <string>

namespace mrrocpp {

namespace mp {

namespace task {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class visualservo_tester : public mrrocpp::mp::task::task
{
public:
	/// utworzenie robotow
	void create_robots(void);
	visualservo_tester(lib::configurator &config);

	virtual ~visualservo_tester();
	void main_task_algorithm(void);
private:
	const std::string config_section_name;
	bool run_vs;
	bool run_conveyor;
	int vs_settle_time;
};

/** @} */

}

}

}

#endif /* MP_T_VISUALSERVO_TESTER_H_ */
