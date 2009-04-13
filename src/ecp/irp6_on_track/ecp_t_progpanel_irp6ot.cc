// ------------------------------------------------------------------------
//   ecp_t_progpanel_irp6ot.cc
//
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
//
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
//#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_pp.h"

#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_pp.h"
//#include "ecp/common/ecp_t_rcsc.h"
#include "ecp/irp6_on_track/ecp_t_progpanel_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

/*int get_object_position(double *coordinates)
{
	int i;

	for(i=1;i<=6;i++) coordinates[i] = 1.0;
	return 0;
}

int create_grab_path(double *object_coordinates, ecp_teach_in_generator generator)
{
	int i;
	double okolica[7], chwyt[7];

	okolica[0]=object_coordinates[0] - 10.0;
	okolica[1]=object_coordinates[1] - 10.0;
	okolica[2]=object_coordinates[2] - 10.0;
	okolica[3]=object_coordinates[3] - 10.0;
	okolica[4]=object_coordinates[4] - 10.0;
	okolica[5]=object_coordinates[5] - 10.0;
	okolica[6]=10.0;

	for(i=0; i<=5; i++) chwyt[i]=object_coordinates[i];
	chwyt[6]=5.0;

	generator.flush_pose_list();
   	generator.create_pose_list_head(XYZ_EULER_ZYZ, 3.0, okolica);
   	generator.insert_pose_list_element(XYZ_EULER_ZYZ, 3.0, object_coordinates);
 	generator.insert_pose_list_element(XYZ_EULER_ZYZ, 3.0, chwyt);

	return 0;
}*/


// KONSTRUKTORY
progpanel::progpanel(lib::configurator &_config) : base(_config)
{
    //	tig = NULL;
    ppg = NULL;
}

// methods for ECP template to redefine in concrete classes
void progpanel::task_initialization(void)
{
    ecp_m_robot = new ecp_irp6_on_track_robot (*this);

    // powolanie czujnikow
    sensor_m[SENSOR_PP] =
        new ecp_mp::sensor::pp (SENSOR_PP, "[vsp_pp_irp6ot]", *this);
    // Konfiguracja czujnika.
    sensor_m.begin()->second->configure_sensor();
    // Stworzenie generatora.
    ppg = new common::generator::progpanel (*this, 16);
    // Przepisanie listy czujnikow.
    ppg->sensor_m = sensor_m;
    sr_ecp_msg->message("ECP loaded");
}

void progpanel::main_task_algorithm(void)
{
	//	ecp_load_file_from_ui(*tig);
	//	ecp_load_file_with_path(*tig, config->return_string_value("trajektoria"));

	//	 Move (*tig);
	ppg->Move();
}

}
} // namespace irp6ot

namespace common {
namespace task {

base* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::progpanel(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


