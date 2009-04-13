// ------------------------------------------------------------------------
//                        		ecp_s.cc		dla QNX6.2
// 
//                     EFFECTOR CONTROL PROCESS (VSP) - metody klasy ecp_mp_sensor()
// 
// Ostatnia modyfikacja: 06.12.2006
// Autor: tkornuta
// ------------------------------------------------------------------------

#include <iostream>
#include <string.h>
#include <unistd.h>

#include "lib/com_buf.h"				// numery bledow

#include "lib/srlib.h"					// klasy bledow
#include "ecp_mp/ecp_mp_sensor.h"				// zawiera klase ecp_mp_sensor
#include "ecp_mp/ecp_mp_s_vis_sac_lx.h"		// zawiera klase ecp_mp_sensor
	
namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/***************************** CONSTRUCTOR ********************************/
vis_sac_lx::vis_sac_lx (SENSOR_ENUM _sensor_name, const char* _section_name, task:: base& _ecp_mp_object):
	base (_sensor_name, _section_name, _ecp_mp_object) {
 
//    printf("ecp_mp_vis_sac_sensor: [vsp_vis_sac_sac]\n");
    // SAC -> uzycie strunktury sizeof(image.sensor_union.camera);
    union_size = sizeof(image.sensor_union.vis_sac);
  
  };//: ecp_mp_vis_sac_sensor

// odebranie odczytu od VSP //
/*
void ecp_mp_vis_sac_lx_sensor::get_reading(){
 	if(read(sd, &from_vsp, sizeof(VSP_ECP_MSG))==-1)
		sr_ecp_msg.message (SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE, VSP_NAME);   
	// jesli odczyt sie powodl, przepisanie pol obrazu z bufora komunikacyjnego do image;
	if(from_vsp.vsp_report == VSP_REPLY_OK)
	{
	
		memcpy( &image.vis_sac, &(from_vsp.comm_image.sensor_union.vis_sac) , union_size);				

	}			

};
*/
	
} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

