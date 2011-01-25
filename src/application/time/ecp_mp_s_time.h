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

#include <ctime>

#include "base/ecp_mp/ecp_mp_sensor.h"				// klasa bazowa ecp_mp_task

namespace boost {
namespace serialization {

template<class Archive>
void serialize(Archive & ar, struct timespec & ts, const unsigned int version)
{
    ar & ts.tv_sec;
    ar & ts.tv_nsec;
}

} // namespace serialization
} // namespace boost

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

typedef sensor<struct timespec> time_sensor_t;

const lib::sensor::SENSOR_t SENSOR_TIME = "SENSOR_TIME";

class time: public time_sensor_t {
  public:
	// Konstruktor czujnika wirtualnego - wywolanie konstruktora klasy bazowej.
 	time (lib::sensor::SENSOR_t _sensor_name, const std::string & _section_name, lib::sr_ecp & _sr_ecp_msg, lib::configurator & _config);

};// end: class time_sensor

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
