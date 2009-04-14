// -------------------------------------------------------------------------
// Proces: 	VIRTUAL SENSOR PROCESS (VSP) 
// Plik:			vsp_sensor.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Deklaracja klasy bazowej vsp_sensor - bazy czujnikow po stronie procesu VSP.
// Autor:		tkornuta
// Data:		09.11.2005
// -------------------------------------------------------------------------

#if !defined(_VSP_SENSOR_H)
#define _VSP_SENSOR_H

#include "lib/sensor.h"
// Konfigurator
#include "lib/configurator.h"

namespace mrrocpp {
namespace vsp {
namespace sensor {

/********** klasa czujnikow po stronie VSP **************/
class base : public lib::sensor {
protected:
	// Flaga - czy czujnik jest skonfigurowany.
	short is_sensor_configured;
	// Flaga - czy jakikolwiek odczyt jest gotowy.
	short is_reading_ready;
	
	
public:
	lib::configurator &config;
	lib::sr_vsp *sr_msg;
	
	char* mrrocpp_network_path;
	
	base (lib::configurator &_config); 
	
	// Metoda uzywana przy wspolpracy nieinteraktywnej.
	virtual void wait_for_event(void);
	
	virtual void terminate(void);

}; // end: class vsp_sensor

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
base* return_created_sensor (lib::configurator &_config);

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

#endif
