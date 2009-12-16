// ------------------------------------------------------------------------
//                        		ecp_s.cc		dla QNX6.2
//
//                     EFFECTOR CONTROL PROCESS (lib::VSP) - metody klasy ecp_mp_sensor()
//
// Ostatnia modyfikacja: 06.12.2006
// Autor: tkornuta
// ------------------------------------------------------------------------

#include "ecp_mp/sensor/ecp_mp_sensor.h"		// zawiera klase ecp_mp_sensor
#include "ecp_mp/sensor/ecp_mp_s_mic.h"		// zawiera klase ecp_mp_mic_sensor

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

mic::mic(lib::SENSOR_t _sensor_name, const char* _section_name, task::task& _ecp_mp_object) :
	sensor(_sensor_name, _section_name, _ecp_mp_object)
{
	//    printf("ecp_mp_mic_sensor: [vsp_mic_sac]\n");
	// SAC -> uzycie strunktury sizeof(image.sensor_union.camera);
	union_size = sizeof(image.sensor_union.mic);
}

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp
