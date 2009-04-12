// ------------------------------------------------------------------------
//   ecp_t_nalewanie.cc
//
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
//
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------

#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include <fstream>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_t_nalewanie.h"
#include "lib/mathtr.h"

namespace mrrocpp {
namespace ecp {
namespace common {

// KONSTRUKTORY
ecp_task_nalewanie::ecp_task_nalewanie(configurator &_config) : ecp_task(_config)
{
    sg = NULL;
}

// methods for ECP template to redefine in concrete classes
void ecp_task_nalewanie::task_initialization(void)
{
    int i;

    if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
    {
        ecp_m_robot = new ecp_irp6_on_track_robot (*this);
    }
    else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
    {
        ecp_m_robot = new irp6p::ecp_irp6_postument_robot (*this);
    }

    sg = new ecp_smooth_generator (*this, true, true);

    sg->load_file_with_path ("../trj/rcsc/irp6ot_sm_ap_2.trj");

    sr_ecp_msg->message("ECP loaded");
}


void ecp_task_nalewanie::main_task_algorithm(void)
{
	/*
	Homog_matrix *mat=new Homog_matrix();
    double qq[7];

	mat->set_xyz_quaternion(1, 2, 3, 0.5, 0.5, 0.5, 0.5);
    mat->get_xyz_quaternion(qq);

    printf("%f, %f, %f, %f, %f, %f, %f\n", qq[0],qq[1],qq[2],qq[3],qq[4],qq[5],qq[6]);
    */

    sg->Move();
}

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_nalewanie(_config);
}

} // namespace common
} // namespace ecp
} // namespace mrrocpp
