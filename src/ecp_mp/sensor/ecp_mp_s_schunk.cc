// ------------------------------------------------------------------------
//                        		ecp_s.cc		dla QNX6.2
// 
//                     EFFECTOR CONTROL PROCESS (lib::VSP) - metody klasy ecp_mp_sensor()
// 
// Ostatnia modyfikacja: 28.05.2003
// Autor: tkornuta
// ------------------------------------------------------------------------

#include "ecp_mp/sensor/ecp_mp_sensor.h"		// zawiera klase ecp_mp_sensor
#include "ecp_mp/sensor/ecp_mp_s_schunk.h"		// zawiera klase ecp_mp_schunk_sensor

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/***************************** CONSTRUCTOR ********************************/
schunk::schunk (lib::SENSOR_ENUM _sensor_name, const char* _section_name, task::task& _ecp_mp_object):
	sensor (_sensor_name, _section_name, _ecp_mp_object) {
  union_size = sizeof(image.sensor_union.force);
}

// inicjacja odczytu dla VSP
void schunk::initiate_reading(){ // by Y - wywalone ze wzgledu na prace nieinteraktywna
}

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp
