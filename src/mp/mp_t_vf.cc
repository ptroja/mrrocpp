// ------------------------------------------------------------------------
// 
//                      MASTER PROCESS (MP) - main()
// 
// ------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <map>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_g_test.h"
#include "mp/mp_t_vf.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "ecp_mp/ecp_mp_s_vis.h"

namespace mrrocpp {
namespace mp {
namespace task {

base* return_created_mp_task (lib::configurator &_config)
{
	return new vis_force(_config);
}

vis_force::vis_force(lib::configurator &_config) : base(_config)
{
}

// methods fo mp template to redefine in concete class
void vis_force::task_initialization(void) 
{
	// Powolanie czujnikow
	sensor_m[SENSOR_FORCE_ON_TRACK] = 
		new ecp_mp::sensor::schunk (SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);

	sensor_m[SENSOR_CAMERA_SA] = 
		new ecp_mp::sensor::vis (SENSOR_CAMERA_SA, "[vsp2_section]", *this);

	// Konfiguracja wszystkich czujnikow	
	for (std::map <SENSOR_ENUM, ::sensor*>::iterator sensor_m_iterator = sensor_m.begin();
	sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}


	usleep(1000*100);
	sr_ecp_msg->message("MP vf loaded");
};


void vis_force::main_task_algorithm(void)
{
	// Utworzenie generatora 
	generator::vis_force vf_gen(*this,8);
	vf_gen.robot_m = robot_m;
	vf_gen.sensor_m = sensor_m;


	vf_gen.Move();

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

