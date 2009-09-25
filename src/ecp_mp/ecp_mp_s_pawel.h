// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (lib::ECP)
// Plik:			ecp_mp_s_vis.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Ogolna struktura obrazow czujnika
// Autor:		tkornuta
// Data:		29.11.2006
// -------------------------------------------------------------------------

#ifndef __ECP_MP_S_PAWEL_H
#define __ECP_MP_S_PAWEL_H

#include "ecp_mp/ecp_mp_sensor.h"				// klasa bazowa sensor

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

/***************** Klasa czujnikow ********************/
class pawel: public sensor{

	private:
	    FILE *f;

	public:
	    // Konstruktor czujnika wirtualnego - wywolanie konstruktora klasy bazowej.
	    pawel (lib::SENSOR_ENUM _sensor_name, const char* _section_name, task::task& _ecp_mp_object);
	    // konstruktor czujnika virtualnego

	    void configure_sensor (void);
	    void initiate_reading (void);
	    void get_reading (void);
};

} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp

#endif
