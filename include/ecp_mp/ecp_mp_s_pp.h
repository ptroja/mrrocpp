// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP) 
// Plik:			ecp_dss.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Struktura - czujnik zlozony z linialow.
// Autor:		tkornuta
// Data:		29.11.2006
// -------------------------------------------------------------------------


#ifndef __ECP_S_PP_H
#define __ECP_S_PP_H



// Klasa bazowa ecp_mp_sensor.
#include "ecp_mp/ecp_mp_sensor.h"				// klasa bazowa sensor

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

// ####################################################################
// ###################   KLASA czujnikow - obsluga  linialow    ########################
// ####################################################################
class ecp_mp_pp_sensor:public ecp_mp_sensor{
  public:
	// Konstruktor czujnika wirtualnego - wywolanie konstruktora klasy bazowej.
 	ecp_mp_pp_sensor (SENSOR_ENUM _sensor_name, const char* _section_name, task:: ecp_mp_task& _ecp_mp_object);
}; 

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp


#endif
