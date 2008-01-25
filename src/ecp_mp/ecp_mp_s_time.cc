// ------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP) 
// Plik:			ecp_fs.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Testowy obraz testowego czujnika VSP
// Autor:		tkornuta
// Data:		04.11.2005
// ------------------------------------------------------------------------

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

// Zawiera klase bazowa ecp_mp_sensor.
#include "ecp_mp/ecp_mp_sensor.h"
// Zawiera klase ecp_mp_time_sensor.
#include "ecp_mp/ecp_mp_s_time.h"

/***************************** CONSTRUCTOR ********************************/
ecp_mp_time_sensor::ecp_mp_time_sensor (SENSOR_ENUM _sensor_name, char* _section_name, ecp_mp_task& _ecp_mp_object):
	ecp_mp_sensor (_sensor_name, _section_name, _ecp_mp_object) {
  union_size = sizeof(image.time);
};//: ecp_mp_time_sensor
