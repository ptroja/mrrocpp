// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP)
// System:	QNX/MRROC++  v. 6.3
// Opis:		metody klasy ecp_mp_sensor dla czujnika z linialami
// Autor:		tkornuta
// Data:		30.11.2006
// -------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include "ecp_mp/ecp_mp_s_pp.h"

/***************************** CONSTRUCTOR ********************************/
ecp_mp_pp_sensor::ecp_mp_pp_sensor(SENSOR_ENUM _sensor_name, char* _section_name, ecp_mp_task& _ecp_mp_object) :
	ecp_mp_sensor(_sensor_name, _section_name, _ecp_mp_object)
{
	// Ustawienie wielkosci przesylanej unii.
	union_size = sizeof(image.pp);
	// Wyzerowanie odczytow.
	for (int i =0; i<3; i++)
		image.pp.joy[i] = 0.0;
	image.pp.active_motors = 0;
}