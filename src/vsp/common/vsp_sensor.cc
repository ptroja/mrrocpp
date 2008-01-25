// ------------------------------------------------------------------------
// Proces:		VIRTUAL SENSOR PROCESS (VSP)
// Plik:            vsp_m_nint.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Interaktywna powloka procesow VSP 
// 
// 	-	interaktywne odczytywanie stanu czujnika rzeczywistego, oczekiwanie na zakonczenie operacji
// 	-	operacje read-write + devctl->(write, read, rw)
// 	-	jednowatkowy
//
// Autor:		tkornuta
// Data:		30.11.2006
// ------------------------------------------------------------------------

#include "lib/srlib.h"

#include "vsp/vsp_sensor.h"				// zawiera deklaracje klasy vsp_sensor + struktury komunikacyjne

void vsp_sensor::wait_for_event(void){};

void vsp_sensor::terminate(void) {
	sr_msg->message("VSP terminated");
}
