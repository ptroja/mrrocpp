/*
 * bclikeregions_task.h
 *
 *  Created on: May 18, 2010
 *      Author: kszkudla
 */

#ifndef BCLIKEREGIONS_TASK_H_
#define BCLIKEREGIONS_TASK_H_

#include <boost/shared_ptr.hpp>
#include "base/ecp/ecp_task.h"
#include "bclikeregions_gen.h"
#include "bclike_gen.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "bcl_types.h"

using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

class bclikeregions_task: public  mrrocpp::ecp::common::task::task{

public:

	bclikeregions_task(mrrocpp::lib::configurator& configurator);
	virtual ~bclikeregions_task();
	void main_task_algorithm(void);

	virtual boost::shared_ptr <bcl_fradia_sensor> get_vsp_fradia();

private:
//	shared_ptr<generator::bclikeregions_gen> gen;
	shared_ptr<generator::bclike_gen> bc_smooth;
	shared_ptr<bcl_fradia_sensor> vsp_fradia;

};

}

}

}

}

#endif /* BCLIKEREGIONS_TASK_H_ */


///*
// * bclikeregions_task.h
// *
// *  Created on: May 18, 2010
// *      Author: kszkudla
// */
//
//#ifndef BCLIKEREGIONS_TASK_H_
//#define BCLIKEREGIONS_TASK_H_
//
//#include "base/mp/mp.h"
//
//using boost::shared_ptr;
//
//namespace mrrocpp {
//
//namespace mp{
//
//namespace task {
//
//class bclikeregions_task: public  task{
//
//private:
//	void configure_edp_force_sensor(bool configure_track, bool configure_postument);
//
//public:
//
//	bclikeregions_task(mrrocpp::lib::configurator& configurator);
//	virtual ~bclikeregions_task();
//	void main_task_algorithm(void);
//
//	//shared_ptr<generator::bclikeregions_gen> gen1;
//
////	ecp::irp6ot::robot ecp_m_robot;
//
//};
//
//}
//
//}
//
//}
//
//
//#endif /* BCLIKEREGIONS_TASK_H_ */
