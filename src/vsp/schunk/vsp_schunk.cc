// -------------------------------------------------------------------------
//                            vsp_s.cc 		dla QNX6.2.1
//
//            Virtual Sensor Process (lib::VSP) - methods for Schunk force/torgue sensor
// Metody klasy VSP
//
// Ostatnia modyfikacja: styczen 2005
// Autor: Yoyek (Tomek Winiarski)
// na podstawie szablonu vsp Tomka Kornuty i programu obslugi czujnika Artura Zarzyckiego
// - wersja do komuniacji z EDP
// -------------------------------------------------------------------------

#include <stdio.h>
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <unistd.h>
#include <time.h>
#include <iostream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "vsp/vsp_schunk.h"

// Konfigurator
#include "lib/configurator.h"

// extern lib::configurator* config;

namespace mrrocpp {
namespace vsp {
namespace sensor {

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
sensor* return_created_sensor (lib::configurator &_config)
{
	return new schunk(_config);
}// : return_created_sensor

// Kontruktor procesu VSP
schunk::schunk(lib::configurator &_config) : sensor(_config)
{
	// Wielkosc unii.
	union_size = sizeof(image.sensor_union.force);

	std::string network_edp_vsp_attach_point =
		config.return_attach_point_name (lib::configurator::CONFIG_SERVER, "edp_vsp_attach_point",
		config.return_string_value("edp_section").c_str());

	ms_nr=0; // numer odczytu z czujnika

	is_sensor_configured=false;	// czujnik niezainicjowany
	is_reading_ready=false;			// nie ma zadnego gotowego odczytu

	ThreadCtl (_NTO_TCTL_IO, NULL);  // nadanie odpowiednich uprawnien watkowi
	// nawiazanie komunikacji z edp

	short tmp = 0;
 	// kilka sekund  (~1) na otworzenie urzadzenia
	while( (edp_vsp_fd = name_open(network_edp_vsp_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL))  < 0)
		if((tmp++)<CONNECT_RETRY)
			delay(CONNECT_DELAY);
		else{
			throw sensor_error (lib::SYSTEM_ERROR, CANNOT_LOCATE_DEVICE);
		};

}

schunk::~schunk(void)
{
   printf("Destruktor VSP\n");
}

/**************************** inicjacja czujnika ****************************/
void schunk::configure_sensor (void)// w obecnej implementacji zeruje poziom odczytow z czujnika w EDP
{

	uint64_t *msg_send_timeout;
	struct sigevent tim_event;
	//	int iw_ret;

	if (to_vsp.parameters==1)
	{
		is_sensor_configured=true;
	//	printf("Sensor initiated\n");
		ap_state=prev_ap_state=next_ap_state=1;
		sr_msg->message ("Sensor initiated by configure_sensor method");
		vsp_edp_command.konfigurowac=1;// zadanie konfiguracji czujnika sily skierowane do EDP
		vsp_edp_command.hdr.type=0;

		msg_send_timeout=new(uint64_t);
		*msg_send_timeout=VSP_MSG_SEND_TIMEOUT_HIGH;
		tim_event.sigev_notify = SIGEV_UNBLOCK;
		TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY ,  &tim_event, msg_send_timeout, NULL );
// 		printf("VSP aaaa\n");
		if (MsgSend(edp_vsp_fd, &vsp_edp_command, sizeof(vsp_edp_command),
		&edp_vsp_reply, sizeof(edp_vsp_reply)) < 0) // by Y
		{
			perror("blad odwolonia VSP_SCHUNK do EDP podczas proby konfiguracji");
			sr_msg->message("blad odwolonia VSP_SCHUNK do EDP podczas proby konfiguracji");
		}
// 		printf("VSP bbbb\n");
		delete msg_send_timeout;

	} else if (to_vsp.parameters>=2) {
		next_ap_state=to_vsp.parameters;
	}
};

void schunk::wait_for_event()
{

	uint64_t *msg_send_timeout;
	struct sigevent tim_event;
	//	int iw_ret;

	// wyslanie i odbior danych z edp
	vsp_edp_command.konfigurowac=0;// zadanie odczytu sily od EDP
	vsp_edp_command.hdr.type=0;

	msg_send_timeout=new(uint64_t);
	*msg_send_timeout=VSP_MSG_SEND_TIMEOUT_HIGH;
	tim_event.sigev_notify = SIGEV_UNBLOCK;

	TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_SEND  | _NTO_TIMEOUT_REPLY ,  &tim_event, msg_send_timeout, NULL );
	if (MsgSend(edp_vsp_fd, &vsp_edp_command, sizeof(vsp_edp_command),
		 &edp_vsp_reply,sizeof(edp_vsp_reply)) < 0) // by Y
	{
		printf("BLAD MsgSend do EDP w VSP_SCHUNK \n");
		delay(1);
		delete msg_send_timeout;
		wait_for_event();
	}

	delete msg_send_timeout;

// 	delay(1); // BY Y DEBUG
};


#define SURFACE_IMPACT 30
#define STATE_4_TO_5 -30
#define STATE_2_TO_4 120

/*************************** inicjacja odczytu ******************************/
void schunk::initiate_reading (void)
{

	if(!is_sensor_configured)
		throw sensor_error (lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);

// 		printf("force: VSP %f\n", edp_vsp_reply.force[2]);

	ap_state=next_ap_state;

	switch (ap_state)
	{
		case 0:
		break;
		case 1:
		break;
		case 2:  // powierzchnia
			if (prev_ap_state!=ap_state)	{ sr_msg->message("VSP Powierzchnia"); }
			if (edp_vsp_reply.force[2]>STATE_2_TO_4) {
				next_ap_state=4;
				}

		break;
		case 3:  // unoszenie (narazie pominiete)
			if (prev_ap_state!=ap_state)	{ sr_msg->message("VSP Unoszenie"); }

		break;
		case 4: // uniesienie
			if (prev_ap_state!=ap_state)	{ sr_msg->message("VSP Uniesienie"); }
				if (edp_vsp_reply.force[2]<(STATE_4_TO_5)) {
			// 	printf("blabla\n");
				next_ap_state=5;
				}

		break;
		case 5: // opuszczanie
			if (prev_ap_state!=ap_state)	{ sr_msg->message("VSP Opuszczanie"); }
						if (edp_vsp_reply.force[2]>SURFACE_IMPACT) {
						//	printf("aaaa: %lf", edp_vsp_reply.force[2]);
							std::cout<<std::endl;
					next_ap_state=2;
				}
		break;
		case 6: // nieaktywne
			if (prev_ap_state!=ap_state)	{ sr_msg->message("VSP Nie wykrywa zdarzen"); }
		break;
		default:
			sr_msg->message("Nieobslugiwany stan aplikacji w initiate reading");
		break;
	}

	prev_ap_state=ap_state;


	for (int i=0;i<=5;i++)
	{
		image.sensor_union.force.rez[i]=edp_vsp_reply.force[i];
// 		image.sensor_union.force.rez[i]=0; // BY Y DEBUG
	}

	image.sensor_union.force.event_type=ap_state; // przepisanie stanbu aplikacji do bufora wysylanego do ECP

	is_reading_ready=true;
	if ((((ms_nr++)%5000)==0)&& (1)){// by Y - debug
		if( clock_gettime( CLOCK_REALTIME , &start[0]) == -1 ) {
		    printf("blad pomiaru czasu");
	     }
	//    printf("%d, %d, %d, %d, %d, %d VSP pomiarow: %d,  czas: %ld\n",image.sensor_union.force.rez[0], image.sensor_union.force.rez[1],
	// 	image.sensor_union.force.rez[2], image.sensor_union.force.rez[3], image.sensor_union.force.rez[4],image.sensor_union.force.rez[5],ms_nr, start[0].tv_sec%100);
	}
};

/***************************** odczyt z czujnika *****************************/
void schunk::get_reading (void)
{


// static long diff;
// static long old_diff;


// clock_gettime( CLOCK_REALTIME , &start[2]);
// diff=start[2].tv_nsec-start[3].tv_nsec;
// start[3].tv_nsec=start[2].tv_nsec;

	if(!is_sensor_configured)
		throw sensor_error (lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	// jezeli chcemy jakikolwiek odczyt	-> is_reading_ready

	if(!is_reading_ready)
	{
// 		printf("DELAY: %d, %d\n", old_diff, diff);

		throw sensor_error (lib::FATAL_ERROR, READING_NOT_READY);

		}
// 	old_diff=diff;
	// ok
	from_vsp.vsp_report= lib::VSP_REPLY_OK;
	// tutaj: czujnik skalibrowany, odczyt dokonany, zapisany w "image", przepisanie wszystkich pol
	// przepisanie do bufora komunikacyjnego
	for(int i=0; i<6; i++)
		from_vsp.comm_image.sensor_union.force.rez[i] = image.sensor_union.force.rez[i];

	from_vsp.comm_image.sensor_union.force.event_type = image.sensor_union.force.event_type; // stan w ktorym jest system wykryty w VSP

	//    sr_msg->message ("VSP Get reading ok");
	is_reading_ready=false;
};

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp
