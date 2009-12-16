// ------------------------------------------------------------------------
//                        		ecp_s.cc		dla QNX6.2
//
//                     EFFECTOR CONTROL PROCESS (lib::VSP) - metody klasy ecp_mp_sensor()
//
// Ostatnia modyfikacja: 06.12.2006
// Autor: tkornuta
// ------------------------------------------------------------------------

#include <iostream>
#include <string.h>
#include <unistd.h>

#include "lib/com_buf.h"				// numery bledow

#include "lib/srlib.h"					// klasy bledow
#include "ecp_mp/sensor/ecp_mp_sensor.h"				// zawiera klase ecp_mp_sensor
#include "ecp_mp/sensor/ecp_mp_s_vis_sac.h"		// zawiera klase ecp_mp_sensor

#warning file not ported to MESSIP yet

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/***************************** CONSTRUCTOR ********************************/
vis_sac::vis_sac (lib::SENSOR_t _sensor_name, const char* _section_name, task::task& _ecp_mp_object):
	sensor (_sensor_name, _section_name, _ecp_mp_object) {

//    printf("ecp_mp_vis_sac_sensor: [vsp_vis_sac_sac]\n");
    // SAC -> uzycie strunktury sizeof(image.sensor_union.camera);
    union_size = sizeof(image.sensor_union.vis_sac);

}//: ecp_mp_vis_sac_sensor

// odebranie odczytu od VSP
void vis_sac::get_reading(){
#if !defined(USE_MESSIP_SRR)
 	if(read(sd, &from_vsp, sizeof(lib::VSP_ECP_MSG))==-1)
		sr_ecp_msg.message (lib::SYSTEM_ERROR, CANNOT_READ_FROM_DEVICE, VSP_NAME);
	// jesli odczyt sie powodl, przepisanie pol obrazu z bufora komunikacyjnego do image;
	if(from_vsp.vsp_report == lib::VSP_REPLY_OK)
	{

		memcpy( &image.sensor_union.vis_sac, &(from_vsp.comm_image.sensor_union.vis_sac) , union_size);				std::cout << "ECP_MP " << sizeof(image.sensor_union.vis_sac) <<" " << sizeof(from_vsp.comm_image.sensor_union.vis_sac) << std::endl;
		for(int i=0; i<16; i++)
			std::cout << from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[i] << " ";
		std::cout << std::endl;

	}
	std::cout << "OUT of block" << std::endl;
#endif
}

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp
