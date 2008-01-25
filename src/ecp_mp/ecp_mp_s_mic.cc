// ------------------------------------------------------------------------
//                        		ecp_s.cc		dla QNX6.2
// 
//                     EFFECTOR CONTROL PROCESS (VSP) - metody klasy ecp_mp_sensor()
// 
// Ostatnia modyfikacja: 06.12.2006
// Autor: tkornuta
// ------------------------------------------------------------------------

#include "ecp_mp/ecp_mp_sensor.h"		// zawiera klase ecp_mp_sensor
#include "ecp_mp/ecp_mp_s_mic.h"		// zawiera klase ecp_mp_mic_sensor

/***************************** CONSTRUCTOR ********************************/
ecp_mp_mic_sensor::ecp_mp_mic_sensor (SENSOR_ENUM _sensor_name, char* _section_name, ecp_mp_task& _ecp_mp_object):
	ecp_mp_sensor (_sensor_name, _section_name, _ecp_mp_object) {
 
//    printf("ecp_mp_mic_sensor: [vsp_mic_sac]\n");
    // SAC -> uzycie strunktury sizeof(image.camera);
    union_size = sizeof(image.mic);
  
};//: ecp_mp_mic_sensor
