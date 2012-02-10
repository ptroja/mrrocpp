/**
 * \file bcl_t_switcher.h
 * \brief Main ECP task file header
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */

#ifndef BCL_T_SWITCHER_H_
#define BCL_T_SWITCHER_H_

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_subtask.h"
#include <boost/shared_ptr.hpp>
#include "ecp_mp_bclike.h"
#include "ecp_mp_message.h"

#include "bcl_types.h"

using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {


///Definition of movement positions
#ifdef IRP6_OT
#ifdef JOINT
#define VEC_SIZE 7
	const double left[] = { 0.0, 0.5, -1.87, 0.100, -0.040, 4.627, -1.57};
	const double right[] = { 0.0, -0.55, -1.37, 0.100, -0.040, 4.627, -1.57};
	const double start[] = { 0.0, 0.0, -1.37, 0.100, -0.040, 4.627, 0.0};
#endif//JOINT

#ifdef EULER
#define VEC_SIZE 6
	const double left[] = { 0.83, 0.5, 0.250, -0.014, 3.100, -0.019};
	const double right[] = {0.83, -0.5, 0.250, 0.007, 3.100, 0.002};
	const double start[] = {0.83, 0.0, 0.250, 0.017, 3.100, 0.012};
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
	const double left[] = { 0.83, 2.45, 0.250, 0.0, 3.100, 0.0};
	const double right[] = {0.83, 1.45, 0.250, 0.0, 3.100, 0.0};
	const double start[] = {0.83, 1.95, 0.250, 0.0, 3.100, 0.0};
#endif //EULER

#endif//IRP6_P


enum BCL_MOTION_DIR{
	LEFT, RIGHT, START
};

class bcl_t_switcher: public common::task::task {
public:
	/**
	 * Class constructor, creating FraDIA sensor, smooth generator, insance
	 * of robot object. Also take care about creating subtasks
	 * @param _config reference to configuration file parser object
	 */
	bcl_t_switcher(lib::configurator &_config);
	/**
	 * Class destructor
	 */
	virtual ~bcl_t_switcher();

	/**
	 * Method for handling communication with MP
	 */
    void mp_2_ecp_next_state_string_handler(void);

    /**
     * Method used by subtask to get access to FraDIA sensor
     * @return pointer to fradia sensor structure
     */
    virtual bcl_fradia_sensor* get_vsp_fradia();

    /**
     * VSP Fradia object pointer
     */
	bcl_fradia_sensor* vsp_fradia;

private:
	ecp_mp_message msg;
};
}

}

}

}

#endif /* BCL_T_SWITCHER_H_ */
