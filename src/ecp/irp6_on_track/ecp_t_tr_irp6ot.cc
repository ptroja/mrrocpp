#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#if defined(__QNXNTO__)
#include <sys/neutrino.h>
#endif /* __QNXNTO__ */

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/ecp_t_tr_irp6ot.h"
// Generator ruchu.
#include "ecp/irp6_on_track/ecp_g_trg.h"
// Warunek startu pomiarow.
#include "ecp/irp6_on_track/ecp_g_rsc.h"
// Czujniki.
#include "ecp_mp/ecp_mp_s_digital_scales.h"
#include "ecp_mp/ecp_mp_s_force.h"
// Konfigurator.
#include "lib/configurator.h"

namespace mrrocpp {
namespace ecp {

namespace common {

extern irp6ot::task::tr* ecp_t;
} // namespace common
namespace irp6ot {
namespace task {


// Kanal komunikacyjny z procesem UI.
name_attach_t * UI_ECP_attach;

// Obiekt zawierajacy sciezki sieciowe.



// Obiekt generator trajektorii.
generator::trajectory_reproduce *trg;
// Obiekt warunek sprawdzania, czy robot sie zatrzymal.
generator::robot_stopped_condition *rsc;

// Flaga uzywana do informowania o koncu pracy.
bool TERMINATE=false;
// Flaga uzywana do informowania o koncu pracy.
bool NEW_TRAJECTORY=false;
// Flaga - polecenie wykonania pierwszego kroku.
bool ZERO_POSITION_MOVE=false;
// Flaga - polecenie wykonania pustego ruchu
bool EMPTY_MOVE=false;
// Flaga - polecenie przerwania ruchu.
bool PAUSE_MOVE=true;
// Flaga - polecenie zakonczenia ruchu.
bool STOP_MOVE=true;

// Zmienna uzywana przy konczeniu watkow.
void* value_ptr;

/********************************** SIGCATCH ********************************/
void tr::catch_signal(int sig)
{
	switch (sig) {
		case SIGTERM:
			// Zakonczenie pracy watkow.
			TERMINATE = true;
			// Koniec pracy czujnikow.
			common::ecp_t->sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR]->terminate();
			// Koniec pracy czujnika.
			if (common::ecp_t->sensor_m.count(lib::SENSOR_FORCE_ON_TRACK)>0)
				common::ecp_t->sensor_m[lib::SENSOR_FORCE_ON_TRACK]->terminate();
			// Zwolnienie pamieci - czujniki.
			delete(common::ecp_t->sensor_m[lib::SENSOR_FORCE_ON_TRACK]);
			delete(common::ecp_t->sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR]);
			// Zwolnienie pamieci - generator.
			delete(trg);
			// Zwolnienie pamieci - warunek.
			delete(rsc);
			// Zwolnienie pamieci - robot.
			delete(common::ecp_t->ecp_m_robot);
			// Odlaczenie nazwy.
			name_detach(UI_ECP_attach, 0);
			common::ecp_t->sr_ecp_msg->message("ECP terminated");
			exit(EXIT_SUCCESS);
			break;
	}
}

/************************ UI COMMUNICATION THREAD ***************************/
void* UI_communication_thread(void* arg)
{
	// Wiadomosc otrzymana z UI.
	lib::UI_ECP_message from_ui_msg;

	// Odsylana liczba makrokrokow.
	int macrosteps;
	// Id nadawcy wiadomosci.
	int rcvid;
	while (!TERMINATE) {
		printf("Elo - Oczekiwanie na wiadomosc!\n");
		// Oczekiwanie na wiadomosc (wcisniety przycisk).
		rcvid = MsgReceive(UI_ECP_attach->chid, &from_ui_msg, sizeof(from_ui_msg), NULL);
		// Jesli zla wiadomosc.
		if (rcvid == -1) {
			perror("UI_communication_thread: Receive failed");
			continue;
		}
		// Jesli nadszedl puls.
		if (rcvid == 0) {
			switch (from_ui_msg.hdr.code) {
				case _PULSE_CODE_DISCONNECT:
					ConnectDetach(from_ui_msg.hdr.scoid);
					break;
				case _PULSE_CODE_UNBLOCK:
					break;
				default:
					break;
			}
			continue;
		}
		printf("Elo - Wiadomosci z QNX IO!\n");
		// Wiadomosci z QNX IO.
		if (from_ui_msg.hdr.type >= _IO_BASE && from_ui_msg.hdr.type <= _IO_MAX) {
			MsgReply(rcvid, EOK, 0, 0);
			continue;
		}
		// Zwykla wiadomosc.
		switch (from_ui_msg.command) {
			case lib::TR_LOAD_TRAJECTORY:
				// Wczytanie trajektorii z pliku.
				trg->load_trajectory(from_ui_msg.filename);
				// Przerwanie oczekiwania na trajektorie.
				NEW_TRAJECTORY = true;
				break;
			case lib::TR_ZERO_POSITION:
				// Wykonanie pierwszego ruchu.
				ZERO_POSITION_MOVE = true;
				// Dalsze dopiero po wcisnieciu START.
				PAUSE_MOVE = false;
				STOP_MOVE = false;
				break;
			case lib::TR_START_MOVE:
				// Rozpoczecie / kontynuacja ruchu.
				PAUSE_MOVE = false;
				STOP_MOVE = false;
				break;
			case lib::TR_PAUSE_MOVE:
				// Chwilowe przerwanie ruchu.
				PAUSE_MOVE = true;
				break;
			case lib::TR_TRY_MOVE_AGAIN:
				// Ponowna proba ruchu.
				PAUSE_MOVE = false;
				break;
			case lib::TR_STOP_MOVE:
				// Przerwanie ruchu - > przerwanie petli do{}while(next).
				STOP_MOVE = true;
				// Przerwanie pauzy.
				PAUSE_MOVE = false;
				break;
			case lib::TR_SAVE_READINGS:
				// Zapis do pliku.
				rsc->save_rse_list(from_ui_msg.filename);
				break;
			case lib::TR_CALIBRATE_DIGITAL_SCALES_SENSOR:
				// Konfiguracja czujnika.
				common::ecp_t->sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR]->configure_sensor();
				break;
			case lib::TR_CALIBRATE_FORCE_SENSOR:
				if (common::ecp_t->sensor_m.count(lib::SENSOR_FORCE_ON_TRACK)>0) {
					// Konfiguracja czujnika sil.
					common::ecp_t->sensor_m[lib::SENSOR_FORCE_ON_TRACK]->configure_sensor();
				}
				break;
			case lib::TR_EXIT:
				// Zakonczenie dzialania procesu.
				TERMINATE = true;
				common::ecp_t->ecp_termination_notice();
				break;
			default:
				fprintf(stderr, "unknown UI command in %s:%d\n", __FILE__, __LINE__);
		}
		if (from_ui_msg.command == lib::TR_LOAD_TRAJECTORY) {
			// Odsylamy liczbe elementow na liscie.
			macrosteps = trg->pose_list_length();
			MsgReply(rcvid, EOK, &macrosteps, sizeof(int));
		} else {
			// Odeslanie pustej odpowiedzi.
			MsgReply(rcvid, EOK, NULL, 0);
		}
	}
	return NULL;
}

/*********************** TRAJECTORY RENDER THREAD *************************/
void* trajectory_reproduce_thread(void* arg)
{
	// Jezeli nie przyszedl rozkaz zakonczenia.
	while (!TERMINATE) {
		// Oczekiwanie na rozkaz wykonania pierwszego kroku.
		while (!ZERO_POSITION_MOVE) {
			usleep(1000*50);
			// Jezeli koniec pracy.
			if (TERMINATE)
				pthread_exit(value_ptr);
		}
		// Przygotowanie danych do ruchu.
		trg->prepare_generator_for_motion();
		rsc->prepare_condition_for_motion();
		// Wykonanie kroku do polozenia zerowego.
		while (1) {
			try
			{ // Zewnetrzna petla try.
				// Oczekiwanie na pozwolenie ruchu.
				while(PAUSE_MOVE)
				{
					usleep(1000*50);
					// Jezeli koniec pracy.
					if(TERMINATE)
					pthread_exit(value_ptr);
				}
				// Jesli wcisnieto STOP -> koniec ruchow.
				if (STOP_MOVE)
				break;
				// Proba wykonania ruchu.
				trg->Move();
				// Ruch wykonano poprawnie.
				break;
			} // end: try zewnetrzne.
			catch (common::generator::generator::ECP_error e)
			{
				// Obsluga bledu.
				trg->dangerous_force_handler(e);
				PAUSE_MOVE = true;
			}
		}
		// Jesli wcisnieto STOP -> koniec ruchow.
		if (STOP_MOVE) {
			// Polecenie oczekiwania na przycisk -> zerowa pozycja.
			ZERO_POSITION_MOVE = false;
			// Skok na poczatek petil.
			continue;
		}
		// Nastepny ruch bez przemieszczenia, tylko zebranie pomiarow.
		EMPTY_MOVE = true;
		// Oczekiwanie na start;
		PAUSE_MOVE = true;
		common::ecp_t->sr_ecp_msg->message("Move to position zero completed. Press START to start measures.");
		do {
			// Oczekiwanie na pozwolenie ruchu.
			while (PAUSE_MOVE) {
				usleep(1000*50);
				// Jezeli koniec pracy.
				if (TERMINATE)
					pthread_exit(value_ptr);
			}
			// Jesli wcisnieto STOP -> koniec ruchow.
			if (STOP_MOVE)
				break;
			// Wewnetrzna pelta try.
			try
			{
				// Jesli robot znajduje sie w pozycji zerowej
				// i pozostalo zebranie pomiarow.
				if(EMPTY_MOVE)
				{
					// Od tej chwili normalne ruchy.
					EMPTY_MOVE = false;
				}
				else
				{
					// Wykonanie ruchu do nastepnego polozenia.

					trg->Move();
				}
				// Jezeli koniec pracy.
				if(TERMINATE)
				pthread_exit(value_ptr);
				// Oczekiwanie na calkowite zatrzymanie robota.
				rsc->Move();
			} // end: try wewnetrzne.
			catch (common::generator::generator::ECP_error e)
			{
				// Obsluga bledu.
				trg->dangerous_force_handler(e);
				// Chwilowe zatrzymanie ruchu.
				PAUSE_MOVE = true;
			} // end: catch
			// Dopoki sa jaskies elementy na liscie makrokrokow.
		} while (trg->is_pose_list_element());
		// Zakonczono trajektorie w wyniku STOP -> koniec ruchow.
		if (STOP_MOVE) {
			// Polecenie oczekiwania na przycisk -> zerowa pozycja.
			ZERO_POSITION_MOVE = false;
			// Skok na poczatek petil.
			continue;
		}
		// Skonczyly sie pozycje na liscie.
		// Dalszy ruch od pierszego kroku.
		ZERO_POSITION_MOVE = false;
		common::ecp_t->sr_ecp_msg->message("Trajectory finished. Press STOP.");
	}
	// koniec dzialania
	pthread_exit(value_ptr);
	return NULL;
}


/********************** TRAJECTORY RENDER WINDOW ************************/
#if !defined(USE_MESSIP_SRR)
void show_trajectory_reproduce_window(int UI_fd)
#else
void show_trajectory_reproduce_window(messip_channel_t * UI_fd)
#endif
{
	int i;
	lib::ECP_message ecp_ui_msg; // Przesylka z ECP do UI
	// Nazwa okna (polecenie otwarcia).
	ecp_ui_msg.hdr.type=0;
	ecp_ui_msg.ecp_message = lib::OPEN_TRAJECTORY_REPRODUCE_WINDOW;
	// Wyslanie polecenia do UI -> otwarcie okna.
#if !defined(USE_MESSIP_SRR)
		if (MsgSend(UI_fd, &ecp_ui_msg, sizeof(lib::ECP_message), NULL, 0) < 0){
#else
		int32_t answer;
		if (messip_send(UI_fd, 0, 0, &ecp_ui_msg, sizeof(lib::ECP_message), &answer, NULL, 0, MESSIP_NOTIMEOUT) < 0){
#endif
		perror("show_trajectory_reproduce_window: Send to UI failed");
		throw common::ECP_main_error(lib::SYSTEM_ERROR, 0);
	}
	// Ustawienie flagi konczenia pracy.
	TERMINATE = false;
	// Ustawienie flagi - nowa trajektoria.
	NEW_TRAJECTORY=false;
	// Ustawienie flagi - polecenie wykonania pierwszego kroku.
	ZERO_POSITION_MOVE=false;
	// Ustawienie flagi - polecenie wykonania pustego ruchu
	EMPTY_MOVE=false;
	// Ustawienie flagi - polecenie przerwania ruchu.
	PAUSE_MOVE=true;
	// Ustawienie flagi - polecenie zakonczenia ruchu.
	STOP_MOVE=true;
	// Atrybuty watku.
	pthread_t tid;
	pthread_attr_t tattr;
	pthread_attr_init( &tattr);
	pthread_attr_setdetachstate( &tattr, PTHREAD_CREATE_DETACHED);
	// Odpalenie watku poruszajacego robotem
	pthread_create(&tid, &tattr, &trajectory_reproduce_thread, (void *)i);
	// Odpalenie watku komunikacji z UI.
	UI_communication_thread((void *)i);
}

// KONSTRUKTORY
tr::tr(lib::configurator &_config) :
	task(_config)
{
	// Stworzenie obiektu robot.
	ecp_m_robot = new robot (*this);
	// Nawiazanie komunikacji z EDP.

	// Dolaczenie globalnej nazwy procesu ECP - kanal do odbioru polecen z UI.
	if ((UI_ECP_attach = name_attach(NULL, "ECP_M_TR", NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		// W razie niepowodzenia.
		throw common::ECP_main_error(lib::SYSTEM_ERROR, NAME_ATTACH_ERROR);
	}

	// Stworznie obiektu - generator uczacy.
	trg = new generator::trajectory_reproduce(*this);

	// Stworzenie obiektu - warunek.
	rsc = new generator::robot_stopped_condition(*this);

	// Stworznie obiektu - czujnik zlozony z linialow.
	//  ini_con->create_vsp ("[vsp_dss]");

	sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR] = new ecp_mp::sensor::digital_scales(lib::SENSOR_DIGITAL_SCALE_SENSOR, "[vsp_dss]", *this);
	//   dss = new ecp_mp_digital_scales_sensor(ini_con->vsp->program_name, ini_con->vsp->node_name,
	// 		ini_con->vsp->resourceman_attach_point, ini_con->config_directories->binaries_network_path,
	// 		argv[1], argv[2], argv[3], "[vsp_dss]" , msg);
	// Konfiguracja czujnika.
	sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR]->configure_sensor();

	// Stworzenie listy czujnikow uzywanych przed instrukcje Wait.
	rsc->sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR] = sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR];

	// Sprawdzanie, czy nalezy uzywac czujnik sily.
	short use_force_sensor = config.return_int_value("use_force_sensor");
	if (use_force_sensor == 1) {
		sr_ecp_msg->message("Using force sensor for move control");
		// Stworzenie obiektu czujnik.
		sensor_m[lib::SENSOR_FORCE_ON_TRACK] = new ecp_mp::sensor::force(lib::SENSOR_FORCE_ON_TRACK, "[vsp_fs]", *this);
		// Konfiguracja czujnika.
		sensor_m[lib::SENSOR_FORCE_ON_TRACK]->configure_sensor();
		// Stworzenie listy czujnikow uzywanych przed instrukcje Move -> glowa = (czujnik sily).
		trg->sensor_m[lib::SENSOR_FORCE_ON_TRACK] = sensor_m[lib::SENSOR_FORCE_ON_TRACK];
		// Dodanie czujnika sily do listy czujnikow uzywanych przez instrukcje WAIT.
		rsc->sensor_m[lib::SENSOR_FORCE_ON_TRACK] = sensor_m[lib::SENSOR_FORCE_ON_TRACK];

	} else {
		sr_ecp_msg->message("Not using force sensor for move control");
		// Pusty czujnik.
		sensor_m.erase(lib::SENSOR_FORCE_ON_TRACK);
	}
	sr_ecp_msg->message("ECP loaded");
}

void tr::main_task_algorithm(void)
{
	// Pokazanie okna .
	show_trajectory_reproduce_window(UI_fd);
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new irp6ot::task::tr(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

