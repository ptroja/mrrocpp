// ------------------------------------------------------------------------
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp/irp6_mechatronika/ecp_r_irp6m.h"

#include "application/teach/ecp_t_teach.h"
#include "ecp/common/generator/ecp_g_teach_in.h"

#include "ecp/common/ecp_robot.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
teach::teach(lib::configurator &_config) : task(_config)
{
    if (config.section_name == ECP_IRP6OT_M_SECTION)
    {
        ecp_m_robot = new irp6ot_m::robot (*this);
    }
    else if (config.section_name == ECP_IRP6P_M_SECTION)
    {
        ecp_m_robot = new irp6p_m::robot (*this);
    }
    else if (config.section_name == ECP_IRP6_MECHATRONIKA_SECTION)
    {
        ecp_m_robot = new irp6m::robot (*this);
    }
    else {
    	fprintf(stderr, "unknown robot \"%s\" in teach task\n", config.section_name.c_str());
    	throw(ecp_robot::ECP_main_error(lib::FATAL_ERROR, 0));
    }

    tig = new generator::teach_in (*this);

    sr_ecp_msg->message("ECP loaded");
}


void teach::main_task_algorithm(void)
{
    switch (ecp_m_robot->robot_name)
    {
    case lib::ROBOT_IRP6OT_M:
        sr_ecp_msg->message("ECP teach irp6ot");
        break;
    case lib::ROBOT_IRP6P_M:
        sr_ecp_msg->message("ECP teach irp6p");
        break;
    default:
        fprintf(stderr, "%s:%d unknown robot type\n", __FILE__, __LINE__);
    }

    if ( operator_reaction ("Teach in? ") )
    {
        tig->flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
        tig->teach (lib::ECP_MOTOR, "Teach-in the trajectory\n");
    }

    if ( operator_reaction ("Save trajectory? ") )
    {
        tig->save_file (lib::ECP_MOTOR);
    }

    if ( operator_reaction ("Load trajectory? ") )
    {
        tig->load_file_from_ui ();
    }

    // Aktualnie petla wykonuje sie jednokrotnie, gdyby MP przejal sterowanie
    // to petle mozna przerwac przez STOP lub przez polecenie lib::END_MOTION wydane
    // przez MP
    //  printf("w ecp for\n");
    tig->Move();
    // 	 printf("w ecp for za move\n");
    // Oczekiwanie na STOP
    ecp_termination_notice();
}

task* return_created_ecp_task (lib::configurator &_config)
{
	return new teach(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

