// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (lib::ECP) 
// Plik:			ecp_mp_s_vis.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Ogolna struktura obrazow czujnika
// Autor:		tkornuta
// Data:		29.11.2006
// -------------------------------------------------------------------------

#ifndef __ECP_MP_S_VIS_H
#define __ECP_MP_S_VIS_H

#include "ecp_mp/sensor/ecp_mp_sensor.h"				// klasa bazowa sensor

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/***************** Klasa czujnikow ********************/
class vis: public sensor{
  public:
	// Konstruktor czujnika wirtualnego - wywolanie konstruktora klasy bazowej.
 	vis (lib::SENSOR_t _sensor_name, const char* _section_name, task::task& _ecp_mp_object);
											// konstruktor czujnika virtualnego
}; 

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
