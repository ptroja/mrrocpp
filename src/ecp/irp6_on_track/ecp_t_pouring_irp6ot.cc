// ------------------------------------------------------------------------
//   ecp_t_pouring_irp6ot.cc - zadanie przelewania, ECP dla IRP6_ON_TRACK
//
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
//
// Ostatnia modyfikacja: 2008
// ------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_pouring.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_st_go.h"
#include "ecp/irp6_on_track/ecp_t_pouring_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace common {

// KONSTRUKTORY
ecp_task_pouring_irp6ot::ecp_task_pouring_irp6ot(configurator &_config) : ecp_task(_config)
{
    sg = NULL;
    tcg = NULL;
}


// methods for ECP template to redefine in concrete classes
void ecp_task_pouring_irp6ot::task_initialization(void)
{
    // the robot is choose dependendat on the section of configuration file sent as argv[4]
    ecp_m_robot = new ecp_irp6_on_track_robot (*this);

    sg = new ecp_smooth_generator (*this, true);
    tcg = new ecp_tool_change_generator(*this, true);

    go_st = new ecp_sub_task_gripper_opening(*this);

    sr_ecp_msg->message("ECP loaded");
};


void ecp_task_pouring_irp6ot::main_task_algorithm(void)
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
                // Stworzenie sciezki do pliku.
                strcpy(path1, mrrocpp_network_path);
                sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
                sg->load_file_with_path (path1);
                //printf("\nON_TRACK ECP_GEN_SMOOTH :%s\n\n", path1);
                delete[] path1;
                //printf("OT po delete\n");
                sg->Move();
                //printf("OT po move\n");
                break;
            case ecp_mp::task::ECP_GEN_POURING:
                tcg->set_tool_parameters(-0.18, 0.0, 0.25);
                tcg->Move();
                break;
            case ecp_mp::task::ECP_END_POURING:
                tcg->set_tool_parameters(0.0, 0.0, 0.25);
                tcg->Move();
                break;
            case ecp_mp::task::GRIP:
                go_st->configure(-0.03, 1000);
                go_st->execute();
                break;
            case ecp_mp::task::LET_GO:
                go_st->configure(0.03, 1000);
                go_st->execute();
                break;
            default:
                break;
            }
            ecp_termination_notice();

        } //end for
}

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_pouring_irp6ot(_config);
}

} // namespace common
} // namespace ecp
} // namespace mrrocpp

