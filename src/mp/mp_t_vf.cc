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

mp_task* return_created_mp_task (configurator &_config)
{
	return new mp_task_vf(_config);
}

mp_task_vf::mp_task_vf(configurator &_config) : mp_task(_config)
{
}

// methods fo mp template to redefine in concete class
void mp_task_vf::task_initialization(void) 
{
	// Powolanie czujnikow
	sensor_m[SENSOR_FORCE_ON_TRACK] = 
		new ecp_mp::sensor::ecp_mp_schunk_sensor (SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);

	sensor_m[SENSOR_CAMERA_SA] = 
		new ecp_mp::sensor::ecp_mp_vis_sensor (SENSOR_CAMERA_SA, "[vsp2_section]", *this);

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


void mp_task_vf::main_task_algorithm(void)
{
	// Utworzenie generatora 
	MP_vf_generator vf_gen(*this,8);
	vf_gen.robot_m = robot_m;
	vf_gen.sensor_m = sensor_m;


	vf_gen.Move();

};
