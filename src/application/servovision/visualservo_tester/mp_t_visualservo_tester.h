/*
 * mp_t_visualservo_tester.h
 *
 *  Created on: May 28, 2010
 *      Author: mboryn
 */

#ifndef MP_T_VISUALSERVO_TESTER_H_
#define MP_T_VISUALSERVO_TESTER_H_

#include "mp/mp.h"

namespace mrrocpp {

namespace mp {

namespace task {

class visualservo_tester : public mrrocpp::mp::task::task
{
public:
	visualservo_tester(lib::configurator &config);
	virtual ~visualservo_tester();
	void main_task_algorithm(void);
};

}

}

}

#endif /* MP_T_VISUALSERVO_TESTER_H_ */
