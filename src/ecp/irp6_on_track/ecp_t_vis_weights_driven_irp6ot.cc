#include <stdio.h>
#include <unistd.h>
#include <map>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
	

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_vis_pb_eol_sac_irp6ot.h"
#include "ecp/irp6_on_track/ecp_vis_pb_eih_irp6ot.h"
#include "ecp/irp6_on_track/ecp_vis_ib_eih_irp6ot.h"
#include "ecp/irp6_on_track/ecp_vis_weights_driven_irp6ot.h"
#include "ecp_mp/ecp_mp_s_vis.h"
#include "ecp/irp6_on_track/ecp_t_vis_weights_driven_irp6ot.h"

#include "ecp_mp/ecp_mp_s_vis_sac_lx.h"

// KONSTRUKTORY
ecp_task_vislx_irp6ot::ecp_task_vislx_irp6ot(configurator &_config) : ecp_task(_config)
{

};

ecp_task_vislx_irp6ot::~ecp_task_vislx_irp6ot(){};


// methods for ECP template to redefine in concrete classes
void ecp_task_vislx_irp6ot::task_initialization(void) 
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
	

	
	// Powolanie czujnikow
		
	sensor_m[SENSOR_CAMERA_SA] = 
		new ecp_mp_vis_sac_lx_sensor (SENSOR_CAMERA_SA, "[vsp_vis]", *this); //change if SENSOR_CAMERA_SA used for nonnn recog (vsp_vis_pbeolsac)
	
	// Konfiguracja wszystkich czujnikow	
	
	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	
	usleep(1000*100);
	sr_ecp_msg->message("ECP weighted vis loaded");
};


void ecp_task_vislx_irp6ot::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP - weighted vis - press start button");
	ecp_wait_for_start();
	ecp_vis_weights_driven_irp6ot ynrlg(*this, 4);
	ynrlg.sensor_m = sensor_m;
	
	pbeolsac = new ecp_vis_pb_eol_sac_irp6ot(*this, 4);
	ynrlg.pbeolsac=pbeolsac;
		
	pbeih = new ecp_vis_pb_eih_irp6ot(*this, 4);
	ynrlg.pbeih=pbeih;
	
	ibeih = new ecp_vis_ib_eih_irp6ot(*this, 4);
	ynrlg.pbeih=pbeih;
	
	   for(;;) { // Wewnetrzna petla nieskoczona


		for(;;) {
		
			sr_ecp_msg->message("NOWA SERIA");
			
			ynrlg.Move();
		
		 }

    // delete(yte_list_head);

     // Oczekiwanie na STOP
     printf("przed wait for stop\n");
     ecp_wait_for_stop();
     break;
   } // koniec: for(;;) wewnetrznej

};

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_vislx_irp6ot(_config);
};
