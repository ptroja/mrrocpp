// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP) 
// Plik:			ecp_mp_s_force.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Definicje klasy ecp_mp_force_sensor - czujnik sily.
// Autor:		tkornuta
// Data:		29.11.2006
// -------------------------------------------------------------------------

#ifndef __ECP_MP_S_FORCE_H
#define __ECP_MP_S_FORCE_H

// klasa bazowa sensor.
#include "ecp_mp/ecp_mp_sensor.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {


// Klasa obrazujaca czujniki sily w systemie MRROC++.
class force: public base{
  public:
	// Konstruktor czujnika wirtualnego - wywolanie konstruktora klasy bazowej.
 	force (lib::SENSOR_ENUM _sensor_name, const char* _section_name, task:: base& _ecp_mp_object);
};// end: class force_sensor

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
