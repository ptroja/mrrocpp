// -------------------------------------------------------------------------
// Proces: 	VIRTUAL SENSOR PROCESS (ECP) 
// Plik:			vsp_fs.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Metody czujnika sily - po stronie procesu VSP.
// Autor:		tkornuta
// Data:		29.11.2006
// -------------------------------------------------------------------------

#include <time.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

// Konfigurator
#include "vsp/vsp_time_sensor.h"


namespace mrrocpp {
namespace vsp {
namespace sensor {

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
base* return_created_sensor (configurator &_config)
{
	return new time(_config);
}// : return_created_sensor


// Konstruktor klasy czujnika wirtualnego, odpowiedzialnego za odczyty z czujnika sily.
time::time(configurator &_config) : base(_config){
	// Wielkosc unii.
	union_size = sizeof(image.sensor_union.time);

	// Czujnik niezainicjowany.
	is_sensor_configured=false;	
	// Nie ma zadnego gotowego odczytu.
	is_reading_ready=false;				
}; // end: vsp_time_sensor

// Metoda sluzaca do konfiguracji czujnika.
void time::configure_sensor (void){// w obecnej implementacji zeruje poziom odczytow z czujnika w EDP
   	is_sensor_configured=true;
}; // end: configure_sensor

// Metoda oczekujaca na dane, otrzymane z czujnika sily (poprzez proces EDP).
void time::wait_for_event(void){
}; // end: wait_for_event

// Metoda dokonujaca przepisania odczytu do obrazu czujnika.
void time::initiate_reading (void){
	// Jesli czujnik nie jest skonfigurowany.
	if(!is_sensor_configured)
	     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	// Odczyt w porzadku.
	is_reading_ready=true;
}; // end: initiate_reading

// Metoda wysyla przepisuje dane z obrazu czujnika do bufora oraz wysyla bufor do procesu oczekujacego na odczyty.
void time::get_reading (void){
	// Jesli czujnik nie jest skonfigurowany.
	if(!is_sensor_configured)
	     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	// Jezeli nie ma nowego odczytu -> wyslanie starego.
	if(!is_reading_ready)
		return;
	// Odczyt w porzadku.
	from_vsp.vsp_report=VSP_REPLY_OK;

	clock_gettime(CLOCK_REALTIME, &from_vsp.comm_image.sensor_union.time.ts);

	// Obacny odczyt nie jest "nowy".
	is_reading_ready=false;
}; // end: get_reading

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp
