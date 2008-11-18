// ------------------------------------------------------------------------
//                        		ecp_s.cc		dla QNX6.2
// 
//                     EFFECTOR CONTROL PROCESS (VSP) - metody klasy ecp_mp_sensor()
// 
// Ostatnia modyfikacja: 28.05.2003
// Autor: tkornuta
// ------------------------------------------------------------------------

#include "ecp_mp/ecp_mp_sensor.h"		// zawiera klase ecp_mp_sensor
#include "ecp_mp/ecp_mp_s_schunk.h"		// zawiera klase ecp_mp_schunk_sensor

/***************************** CONSTRUCTOR ********************************/
ecp_mp_schunk_sensor::ecp_mp_schunk_sensor (SENSOR_ENUM _sensor_name, const char* _section_name, ecp_mp_task& _ecp_mp_object):
	ecp_mp_sensor (_sensor_name, _section_name, _ecp_mp_object) {
  union_size = sizeof(image.sensor_union.force);
}

// inicjacja odczytu dla VSP
void ecp_mp_schunk_sensor::initiate_reading(){ // by Y - wywalone ze wzgledu na prace nieinteraktywna
}
