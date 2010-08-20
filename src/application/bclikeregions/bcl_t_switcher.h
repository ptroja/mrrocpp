/*
 * bcl_t_switcher.h
 *
 *  Created on: Jul 6, 2010
 *      Author: kszkudla
 */

#ifndef BCL_T_SWITCHER_H_
#define BCL_T_SWITCHER_H_

#include "base/ecp/ecp_task.h"
#include "bclike_smooth.h"
#include <boost/shared_ptr.hpp>
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp_mp_bclike.h"
#include "ecp_st_smooth_move.h"
#include "bcl_message.h"
#include "../../generator/ecp/ecp_g_constant_velocity.h"

#include "ecp_mp_st_smooth_move.h"

#include "bcl_types.h"

using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

#ifdef IRP6_OT
#define VEC_SIZE 7
#ifdef JOINT
	const double left[] = { 0.0, 0.5, -1.87, 0.100, -0.040, 4.627, -1.57};
	const double right[] = { 0.0, -0.55, -1.37, 0.100, -0.040, 4.627, -1.57};
	const double start[] = { 0.0, 0.0, -1.37, 0.100, -0.040, 4.627, 0.0};
#endif//JOINT

#ifdef EULER
	const double left[] = { 0.83, 0.5, 0.100, -0.59, 3.035, -1.055};
	const double right[] = {0.83, -0.5, 0.100, -0.59, 3.035, -1.055};
	const double start[] = {0.83, -0.02, 0.100, -0.59, 3.035, -1.055};
#endif //EULER

#endif//IRP6_OT

#ifdef IRP6_P
#define VEC_SIZE 6
#ifdef JOINT
	const double left[] = { 0.5, -1.87, 0.100, -0.040, 4.627, -1.57};
	const double right[] = {-0.55, -1.37, 0.100, -0.040, 4.627, -1.57};
	const double start[] = {0.0, -1.37, 0.100, -0.040, 4.627, 0.0};
#endif //JOINT

#ifdef EULER
	const double left[] = { 0.83, 2.45, 0.100, -0.59, 3.035, -1.055};
	const double right[] = {0.83, 1.45, 0.100, -0.59, 3.035, -1.055};
	const double start[] = {0.83, 1.95, 0.100, -0.59, 3.035, -1.055};
#endif //EULER

#endif//IRP6_P


enum BCL_MOTION_DIR{
	LEFT, RIGHT, START
};

class bcl_t_switcher: public mrrocpp::ecp::common::task::task {
public:
	bcl_t_switcher(lib::configurator &_config);
	virtual ~bcl_t_switcher();

    void mp_2_ecp_next_state_string_handler(void);
    virtual bcl_fradia_sensor* get_vsp_fradia();

	bcl_fradia_sensor* vsp_fradia;

private:
	shared_ptr<generator::newsmooth> bc_smooth;

	std::vector<double> vec;

	std::vector<mrrocpp_regions> reading;

	bcl_message msg;
};
}

}

}

}

#endif /* BCL_T_SWITCHER_H_ */
