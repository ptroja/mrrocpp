// -------------------------------------------------------------------------
// Proces: 	VIRTUAL SENSOR PROCESS (ECP) 
// Plik:			vsp_fs.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Metody czujnika sily - po stronie procesu VSP.
// Autor:		tkornuta
// Data:		29.11.2006
// -------------------------------------------------------------------------

#include <sys/neutrino.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"

// Konfigurator
#include "lib/configurator.h"
#include "vsp/vsp_force_sensor.h"


namespace mrrocpp {
namespace vsp {
namespace sensor {


// Zmienne konfiguracyjne.
// edp_schunk_config* edp_schunk_c;
// extern ini_configs* ini_con;
// extern lib::configurator* config;


// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
base* return_created_sensor (lib::configurator &_config)
{
	return new force(_config);
}// : return_created_sensor


// Konstruktor klasy czujnika wirtualnego, odpowiedzialnego za odczyty z czujnika sily.
force::force(lib::configurator &_config) : base(_config){
	// Wielkosc unii.
	union_size = sizeof(image.sensor_union.force);

	char* network_edp_vsp_attach_point = 
		config.return_attach_point_name (lib::configurator::CONFIG_SERVER, "edp_vsp_attach_point", 
		config.return_string_value("edp_section"));

 	ms_nr=0; // numer odczytu z czujnika

	// Czujnik niezainicjowany.
	is_sensor_configured=false;	
	// Nie ma zadnego gotowego odczytu.
	is_reading_ready=false;				
	// Wczytanie konfiguracji.
	// ini_con->create_edp_irp6_on_track  ( ini_con->vsp->edp_section);	
	// Nadanie odpowiednich uprawnien watkowi.
	ThreadCtl (_NTO_TCTL_IO, NULL);  
	// Nawiazanie komunikacji z edp.
	short tmp = 0;
 	// Kilka sekund  (~1) na otworzenie urzadzenia.
	while( (edp_vsp_fd = name_open(network_edp_vsp_attach_point, NAME_FLAG_ATTACH_GLOBAL))  < 0)
		if((tmp++)<CONNECT_RETRY)
			delay(CONNECT_DELAY);
		else{
			throw sensor_error (SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
		};
	
	delete [] network_edp_vsp_attach_point;
	}; // end: vsp_force_sensor

// Metoda sluzaca do konfiguracji czujnika.
void force::configure_sensor (void){// w obecnej implementacji zeruje poziom odczytow z czujnika w EDP
   	is_sensor_configured=true;
	vsp_edp_command.konfigurowac=1;// zadanie konfiguracji czujnika sily skierowane do EDP
	vsp_edp_command.hdr.type=0;
	if (MsgSend(edp_vsp_fd, &vsp_edp_command, sizeof(vsp_edp_command), &edp_vsp_reply, sizeof(edp_vsp_reply)) < 0){
		printf("blad odwolonia vsp do edp podczas proby konfiguracji\n");
	}else
		sr_msg->message ("Sensor configured");
	}; // end: configure_sensor

// Metoda oczekujaca na dane, otrzymane z czujnika sily (poprzez proces EDP).
void force::wait_for_event(void){
	// Zadanie odczytu sily od EDP.
	vsp_edp_command.konfigurowac=0;
	vsp_edp_command.hdr.type=0;
	// Wyslanie i odbior danych z edp.
	while(MsgSend(edp_vsp_fd, &vsp_edp_command, sizeof(vsp_edp_command), &edp_vsp_reply, sizeof(edp_vsp_reply)) < 0){
		delay(1);
		};
	}; // end: wait_for_event

// Metoda dokonujaca przepisania odczytu do obrazu czujnika.
void force::initiate_reading (void){
	// Jesli czujnik nie jest skonfigurowany.
	if(!is_sensor_configured)
	     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	// Przepisanie odczytu do obrazu czujnika.
	memcpy(image.sensor_union.force.rez, edp_vsp_reply.force, 6*sizeof(double));
	// Odczyt w porzadku.
	is_reading_ready=true;
	// Pomiar czasu.
	if ((((ms_nr++)%5000)==0)&& (1)){
		if( clock_gettime( CLOCK_REALTIME , &start[0]) == -1 ) {
		    printf("blad pomiaru czasu");
		     }; 
		};
	}; // end: initiate_reading

// Metoda wysyla przepisuje dane z obrazu czujnika do bufora oraz wysyla bufor do procesu oczekujacego na odczyty.
void force::get_reading (void){
	// Jesli czujnik nie jest skonfigurowany.
	if(!is_sensor_configured)
	     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	// Jezeli nie ma nowego odczytu -> wyslanie starego.
	if(!is_reading_ready)
		return;
	// Odczyt w porzadku.
	from_vsp.vsp_report= lib::VSP_REPLY_OK;
	// Przepisanie pomiarow z obrazu czujnika do bufora komunikacyjnego.
	memcpy(from_vsp.comm_image.sensor_union.force.rez, image.sensor_union.force.rez, 6*sizeof(double));
	// Obacny odczyt nie jest "nowy".
     is_reading_ready=false;
	}; // end: get_reading

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

