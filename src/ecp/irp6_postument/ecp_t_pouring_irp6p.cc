// ------------------------------------------------------------------------
//   ecp_t_pouring_irp6p.cc - zadanie przelewania, ECP dla IRP6_POSTUMENT
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2008
// ------------------------------------------------------------------------


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <map>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_pouring.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/irp6_postument/ecp_t_pouring_irp6p.h"


namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {


// KONSTRUKTORY
pouring::pouring(lib::configurator &_config) : task(_config)
{
    sg = NULL;
}


void pouring::task_initialization(void)
{
    ecp_m_robot = new ecp_irp6_postument_robot (*this);

    // Powolanie czujnikow
    //	sensor_m[lib::SENSOR_FORCE_POSTUMENT] =
    //		new ecp_mp_schunk_sensor (lib::SENSOR_FORCE_POSTUMENT, "[vsp_force_irp6p]", *this);

    /*	// Konfiguracja wszystkich czujnikow
    	for (ecp_mp::sensor_map::iterator sensor_m_iterator = sensor_m.begin();
    		sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
    	{
    		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
    		sensor_m_iterator->second->configure_sensor();
    	}

    	usleep(1000*100);
    */
    sg = new common::generator::smooth (*this, true);

    go_st = new common::task::ecp_sub_task_gripper_opening(*this);

    sr_ecp_msg->message("ECP loaded");
}


void pouring::main_task_algorithm(void)
{
   for(;;)
    {
    	sr_ecp_msg->message("Waiting for MP order");

    	get_next_state ();

    	sr_ecp_msg->message("Order received");

    	switch ( (ecp_mp::task::POURING_ECP_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state)
    	{
    		case ecp_mp::task::ECP_GEN_SMOOTH:
    		{
    			std::string path(mrrocpp_network_path);
    			path += mp_command.ecp_next_state.mp_2_ecp_next_state_string;
    			sg->load_file_with_path (path.c_str());
    			sg->Move();
    			break;
    		}
    		case ecp_mp::task::GRIP:
    			go_st->configure(-0.018, 1000);
    			go_st->execute();
    			break;
    		case ecp_mp::task::LET_GO:
    			go_st->configure(0.015, 2000);
    			go_st->execute();
    			break;
    		case ecp_mp::task::WEIGHT:
    			printf("force0: %f\n", sensor_m.begin()->second->image.sensor_union.force.rez[0]);
    			printf("force1: %f\n", sensor_m.begin()->second->image.sensor_union.force.rez[0]);
    			printf("force2: %f\n", sensor_m.begin()->second->image.sensor_union.force.rez[0]);
    			break;
    		default:
    			break;
    	} // end switch
    	ecp_termination_notice();

    } //end for
}

}
} // namespace irp6p

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6p::task::pouring(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

