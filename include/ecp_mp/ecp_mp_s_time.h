// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP) 
// Plik:			ecp_mp_s_time.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Definicje klasy ecp_mp_time_sensor - czujnik czasu
// Autor:		ptrojane
// Data:		19.04.2007
// -------------------------------------------------------------------------

#ifndef __ECP_MP_S_FORCE_H
#define __ECP_MP_S_FORCE_H

#include "ecp_mp/ecp_mp_sensor.h"				// klasa bazowa ecp_mp_task

// Klasa obrazujaca czujniki sily w systemie MRROC++.
class ecp_mp_time_sensor: public ecp_mp_sensor{
  public:
	// Konstruktor czujnika wirtualnego - wywolanie konstruktora klasy bazowej.
 	ecp_mp_time_sensor (SENSOR_ENUM _sensor_name, char* _section_name, ecp_mp_task& _ecp_mp_object);

};// end: class time_sensor

#endif
