// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (lib::ECP) 
// Plik:			ecp_mp_s_time.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Definicje klasy ecp_mp_time_sensor - czujnik czasu
// Autor:		ptrojane
// Data:		19.04.2007
// -------------------------------------------------------------------------

#ifndef __ECP_MP_S_FORCE_H
#define __ECP_MP_S_FORCE_H

#include "ecp_mp/ecp_mp_sensor.h"				// klasa bazowa ecp_mp_task

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

// Klasa obrazujaca czujniki sily w systemie MRROC++.
class time: public base{
  public:
	// Konstruktor czujnika wirtualnego - wywolanie konstruktora klasy bazowej.
 	time (lib::SENSOR_ENUM _sensor_name, const char* _section_name, task:: base& _ecp_mp_object);

};// end: class time_sensor

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
