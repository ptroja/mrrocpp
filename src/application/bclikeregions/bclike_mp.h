/*
 * bclike_mp.h
 *
 *  Created on: 06-07-2010
 *      Author: kszkudla
 */

#ifndef BCLIKE_MP_H_
#define BCLIKE_MP_H_

//#include "base/mp/mp.h"
#include "base/mp/mp_task.h"
#include "ecp_mp_bclike.h"
#include "bcl_message.h"
#include "bcl_types.h"

#include "ecp_mp_st_smooth_move.h"

namespace mrrocpp {

namespace mp {

namespace task {


class bclike_mp: public task {
public:
	bclike_mp(lib::configurator &_config);
	virtual ~bclike_mp();

	void main_task_algorithm(void);

private:
	ecp::common::bcl_message msg;
	ecp::common::task::fradia_regions reg;
	std::vector<double> pos;

	std::vector<std::pair<ecp::common::task::mrrocpp_regions, bool> > regions;

	//Second FraDIA task definition
	std::string second_task;


};

}

}

}

#endif /* BCLIKE_MP_H_ */
