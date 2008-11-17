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

#include "common/sensor.h"

// Wskaznik na lacze z SR
extern sr_vsp *sr_msg;

/********** klasa czujnikow po stronie VSP **************/
class vsp_sensor : public sensor {
protected:
	// Flaga - czy czujnik jest skonfigurowany.
	short is_sensor_configured;
	// Flaga - czy jakikolwiek odczyt jest gotowy.
	short is_reading_ready;
	
public:
	char* mrrocpp_network_path;
	// Metoda uzywana przy wspolpracy nieinteraktywnej.
	virtual void wait_for_event(void);
	
	virtual void terminate(void);

}; // end: class vsp_sensor

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
vsp_sensor* return_created_sensor (void);

#endif
