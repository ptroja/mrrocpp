// ------------------------------------------------------------------------
//   ecp_t_pouring_irp6p.cc - zadanie przelewania, ECP dla IRP6_POSTUMENT
//
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
//
// Ostatnia modyfikacja: 2008
// ------------------------------------------------------------------------


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <map>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_pouring.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/irp6_postument/ecp_t_pouring_irp6p.h"


namespace mrrocpp {
namespace ecp {
namespace irp6p {


// KONSTRUKTORY
ecp_task_pouring_irp6p::ecp_task_pouring_irp6p(configurator &_config) : ecp_task(_config)
{
    sg = NULL;
}


void ecp_task_pouring_irp6p::task_initialization(void)
{
    ecp_m_robot = new ecp_irp6_postument_robot (*this);

    // Powolanie czujnikow
    //	sensor_m[SENSOR_FORCE_POSTUMENT] =
    //		new ecp_mp_schunk_sensor (SENSOR_FORCE_POSTUMENT, "[vsp_force_irp6p]", *this);

    /*	// Konfiguracja wszystkich czujnikow
    	for (std::map <SENSOR_ENUM, ::sensor*>::iterator sensor_m_iterator = sensor_m.begin();
    		sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
    	{
    		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
    		sensor_m_iterator->second->configure_sensor();
    	}

    	usleep(1000*100);
    */
    sg = new common::ecp_smooth_generator (*this, true);

    go_st = new common::task::ecp_sub_task_gripper_opening(*this);

    sr_ecp_msg->message("ECP loaded");
}


void ecp_task_pouring_irp6p::main_task_algorithm(void)
{

    int size;
    char * path1;

    for(;;)
    {
    	sr_ecp_msg->message("Waiting for MP order");

    	get_next_state ();

    	sr_ecp_msg->message("Order received");

    	switch ( (ecp_mp::task::POURING_ECP_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state)
    	{
    		case ecp_mp::task::ECP_GEN_SMOOTH:
    			size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.ecp_next_state.mp_2_ecp_next_state_string);
    			path1 = new char[size];
    			strcpy(path1, mrrocpp_network_path);
    			sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
    			sg->load_file_with_path (path1);
    			delete[] path1;
    			sg->Move();
    			break;
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

} // namespace irp6p

namespace common {
namespace task {

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new irp6p::ecp_task_pouring_irp6p(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

