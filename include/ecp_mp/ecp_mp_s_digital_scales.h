// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP) 
// Plik:			ecp_dss.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Struktura - czujnik zlozony z linialow.
// Autor:		tkornuta
// Data:		29.11.2006
// -------------------------------------------------------------------------

#ifndef __ECP_DSS_H
#define __ECP_DSS_H

#include "ecp_mp/ecp_mp_sensor.h"				// klasa bazowa sensor

// ####################################################################
// ###################   KLASA czujnikow - obsluga  linialow    ########################
// ####################################################################
class ecp_mp_digital_scales_sensor:public ecp_mp_sensor{
  public:
	// Konstruktor czujnika wirtualnego - wywolanie konstruktora klasy bazowej.
 	ecp_mp_digital_scales_sensor (SENSOR_ENUM _sensor_name, char* _section_name, ecp_mp_task& _ecp_mp_object);
}; 

#endif
