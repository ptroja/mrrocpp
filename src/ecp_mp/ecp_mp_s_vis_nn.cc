// ------------------------------------------------------------------------
//                        		ecp_s.cc		dla QNX6.2
//
//                     EFFECTOR CONTROL PROCESS (VSP) - metody klasy ecp_mp_sensor()
//
// Ostatnia modyfikacja: 06.12.2006
// Autor: tkornuta
// ------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>

#include "ecp_mp/ecp_mp_sensor.h"		// zawiera klase ecp_mp_sensor
#include "ecp_mp/ecp_mp_s_vis_nn.h"		// zawiera klase ecp_mp_vis_nn_sensor

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/***************************** CONSTRUCTOR ********************************/
vis_nn::vis_nn (SENSOR_ENUM _sensor_name, const char* _section_name, task:: base& _ecp_mp_object):
	base (_sensor_name, _section_name, _ecp_mp_object) {
	if (strcmp(_section_name, "[vsp_vis_eih]") == 0)
	{
		union_size = sizeof(image.sensor_union.cube_face);
	} else {
		//    printf("ecp_mp_vis_sensor: [vsp_vis_sac]\n");
		// SAC -> uzycie strunktury sizeof(image.sensor_union.camera);
		union_size = sizeof(image.sensor_union.camera);
	}//: if
}//: ecp_mp_vis_sensor

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp
