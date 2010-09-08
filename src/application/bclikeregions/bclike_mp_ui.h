/*
 * bclike_mp.h
 *
 *  Created on: 06-07-2010
 *      Author: kszkudla
 */

#ifndef BCLIKE_MP_UI_H_
#define BCLIKE_MP_UI_H_

//#include "base/mp/mp.h"
#include "base/mp/mp_task.h"
#include "ecp_mp_bclike.h"
#include "ecp_mp_message.h"
#include "bcl_types.h"

namespace mrrocpp {

namespace mp {

namespace task {


class bclike_mp_ui: public task {
public:
	bclike_mp_ui(lib::configurator &_config);
	virtual ~bclike_mp_ui();

	void main_task_algorithm(void);
	virtual void create_robots(void);

private:
	ecp::common::ecp_mp_message msg;
	ecp::common::task::fradia_regions reg;
	std::vector<double> pos;

	std::vector<std::pair<ecp::common::task::mrrocpp_regions, bool> > regions;

	//Second FraDIA task definition
	std::string second_task;

	int movement_type;


};

}

}

}

#endif /* BCLIKE_MP_UI_H_ */
