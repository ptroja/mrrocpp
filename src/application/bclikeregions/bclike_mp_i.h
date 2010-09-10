/**
 * \file bclike_mp_i.h
 * \brief MP process class: INTERRUPTED_MOVE
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */


#ifndef BCLIKE_MP_I_H_
#define BCLIKE_MP_I_H_

//#include "base/mp/mp.h"
#include "base/mp/mp_task.h"
#include "ecp_mp_bclike.h"
#include "ecp_mp_message.h"
#include "bcl_types.h"

namespace mrrocpp {

namespace mp {

namespace task {


class bclike_mp_i: public task {
public:
	/**
	 * Main Process class constructor
	 * @param _config reference to configuration file parser object
	 */
	bclike_mp_i(lib::configurator &_config);
	/**
	 * Class destructor
	 */
	virtual ~bclike_mp_i();

	/**
	 * Class main method responsible for switching between subtask
	 * depending on received data
	 */
	void main_task_algorithm(void);
	/**
	 * Create objects of robots used in task
	 */
	virtual void create_robots(void);

private:
	ecp::common::ecp_mp_message msg;
	ecp::common::task::fradia_regions reg;
	std::vector<double> pos;

	std::vector<std::pair<ecp::common::task::mrrocpp_regions, bool> > regions;


};

}

}

}

#endif /* BCLIKE_MP_I_H_ */
