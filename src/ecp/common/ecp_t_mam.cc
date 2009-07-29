// ------------------------------------------------------------------------
//   ((ecp_task_mam*)ecp_t)_tran.cc - przezroczyste wersja dla dowolnego z robotow
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------

#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/common/ecp_t_mam.h"
// Generator ruchu.
#include "ecp/common/ecp_g_mam.h"
// Czujnik.
#include "ecp_mp/ecp_mp_s_digital_scales.h"

#include "ecp/common/ECP_main_error.h"

namespace mrrocpp {
namespace ecp {
namespace common {

// Obiekt zawierajacy sciezki sieciowe.
extern task::task* ecp_t;

namespace task {

// Zmienne do komunikacji.
extern name_attach_t *ecp_attach;
// Kanal komunikacyjny z procesem UI.
name_attach_t * UI_ECP_attach;



// Obiekt generator trajektorii.
generator::manual_moves_automatic_measures *mam_gen;

// Flaga uzywana do informowania o koncu pracy.
bool TERMINATE=false;
// Flaga uzywana do zatrzymywania/uruchamiania zbierania pomiarow.
bool START_MEASURES=false;

// Zmienna uzywana przy konczeniu watkow.
void* value_ptr;

/********************************** SIGCATCH ********************************/
void mam::catch_signal(int sig)
{
	switch (sig) {
		case SIGTERM:
			// Zakonczenie pracy watkow.
			TERMINATE = true;
			// Koniec pracy czujnikow.
			(((mam*)ecp_t))->sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR]->terminate();
			// Zwolnienie pamieci - czujnik.
			delete(((mam*)ecp_t)->sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR]);

			// Zwolnienie pamieci - generator.
			delete(mam_gen);
			// Zwolnienie pamieci - robot.
			delete(((mam*)ecp_t)->ecp_m_robot);
			// Odlaczenie nazwy.
			name_detach(UI_ECP_attach, 0);
			((mam*)ecp_t)->sr_ecp_msg->message("ECP terminated");
			exit(EXIT_SUCCESS);
			break;
	}
}

/************************ UI COMMUNICATION THREAD ***************************/
void* UI_communication_thread(void* arg)
{
	// Wiadomosc otrzymana z UI.
	lib::UI_ECP_message from_ui_msg;

	// Id nadawcy wiadomosci.
	int rcvid;
	while (!TERMINATE) {
		// Oczekiwanie na wiadomosc (wcisniety przycisk).
		rcvid = MsgReceive(UI_ECP_attach->chid, &from_ui_msg, sizeof(from_ui_msg), NULL);
		// Jesli zla wiadomosc.
		if (rcvid == -1) {
			perror("UI_communication_thread: Receive failed\n");
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
		// Wiadomosci z QNX IO.
		if (from_ui_msg.hdr.type >= _IO_BASE && from_ui_msg.hdr.type <= _IO_MAX) {
			MsgReply(rcvid, EOK, 0, 0);
			continue;
		}
		// Zwykla wiadomosc.
		switch (from_ui_msg.command) {
			case lib::MAM_START:
				// Rozpoczecie wykonywania pomiarow.
				START_MEASURES = true;
				break;
			case lib::MAM_STOP:
				// Zakonczenie wykonywania pomiarow.
				START_MEASURES = false;
				break;
			case lib::MAM_SAVE:
				// Zapis do pliku.
				mam_gen->save_mam_list(from_ui_msg.filename);
				break;
			case lib::MAM_CLEAR:
				// Oproznienie listy z pomiarami.
				mam_gen->flush_mam_list();
				break;
			case lib::MAM_CALIBRATE:
				// Konfiguracja czujnika.
				((mam*)ecp_t)->sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR]->configure_sensor();
				break;
			case lib::MAM_EXIT:
				// Zakonczenie dzialania procesu.
				TERMINATE = true;
				break;
			default:
				fprintf(stderr, "unknown MAM command in %s:%d\n", __FILE__, __LINE__);
				break;
		}
		// Odeslanie pustej odpowiedzi.
		MsgReply(rcvid, EOK, NULL, 0);
	}
	return NULL;
}

/*************************** MEASURES THREAD ******************************/
void* measures_thread(void* arg)
{
	// Jezeli nie przyszedl rozkaz zakonczenia.
	while (!TERMINATE) {
		// Oczekiwanie na rozkaz wykonania pierwszego kroku.
		while (!START_MEASURES) {
			usleep(1000*50);
			// Jezeli koniec pracy.
			if (TERMINATE)
				pthread_exit(value_ptr);
		}
		// Zebranie pomiarow co np. 300 ms.
		mam_gen->Move();
		// Oraz cale porownanie.
		usleep(1000*300);
		// Oraz odswierzenie okna.
		//        ((ecp_task_mam*)ecp_t)->sr_ecp_msg->message("Tak sobie iteruje.");
	}
	// koniec dzialania
	pthread_exit(value_ptr);
	return NULL;
}

/*************************** SHOW MAM WINDOW *****************************/
void show_mam_window(int UI_fd)
{
	int i;
	lib::ECP_message ecp_ui_msg; // Przesylka z ECP do UI
	// Nazwa okna (polecenie otwarcia).
	ecp_ui_msg.hdr.type=0;
	ecp_ui_msg.ecp_message = lib::MAM_OPEN_WINDOW;
	// Wyslanie polecenia do UI -> otwarcie okna.
	if (MsgSend(UI_fd, &ecp_ui_msg, sizeof(lib::ECP_message), NULL, 0) < 0) {
		perror("show_mam_window: Send to UI failed");
		throw ECP_main_error(lib::SYSTEM_ERROR, 0);
	}
	// Ustawienie flagi konczenia pracy.
	TERMINATE = false;
	// Ustawienie flagi zbierania pomiarow.
	START_MEASURES=false;
	// Atrybuty watku.
	pthread_attr_t tattr;
	pthread_attr_init( &tattr);
	pthread_attr_setdetachstate( &tattr, PTHREAD_CREATE_DETACHED);
	// Odpalenie watku poruszajacego robotem
	pthread_create( NULL, &tattr, &measures_thread, (void *)i);
	// Odpalenie watku komunikacji z UI.
	UI_communication_thread((void *)i);
}

// KONSTRUKTORY
mam::mam(lib::configurator &_config) :
	task(_config)
{
}

mam::~mam()
{
}

// methods for ECP template to redefine in concrete classes
void mam::task_initialization(void)
{
	// the robot is choose dependendant on the section of configuration file sent as argv[4]
	if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0) {
		ecp_m_robot = new irp6ot::ecp_irp6_on_track_robot (*this);
	} else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0) {
		ecp_m_robot = new irp6p::ecp_irp6_postument_robot (*this);
	}

	// Dolaczenie globalnej nazwy procesu ECP - kanal do odbioru polecen z UI.
	if ((UI_ECP_attach = name_attach(NULL, "ECP_M_MAM", NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		// W razie niepowodzenia.
		throw ECP_main_error(lib::SYSTEM_ERROR, NAME_ATTACH_ERROR);
	}

	// Stworznie obiektu - generator uczacy.
	mam_gen = new generator::manual_moves_automatic_measures(*this, 8);

	// Stworznie obiektu - czujnik zlozony z linialow.
	sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR] = new ecp_mp::sensor::digital_scales(lib::SENSOR_DIGITAL_SCALE_SENSOR, "[vsp_dss]", *this);
	// Konfiguracja czujnika.
	sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR]->configure_sensor();

	// Stworzenie listy czujnikow.
	mam_gen->sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR] = sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR];

	switch (ecp_m_robot->robot_name) {
		case lib::ROBOT_IRP6_ON_TRACK:
			sr_ecp_msg->message("ECP mam irp6ot loaded");
			break;
		case lib::ROBOT_IRP6_POSTUMENT:
			sr_ecp_msg->message("ECP mam irp6p loaded");
			break;
		default:
			break;
	}
}

void mam::main_task_algorithm(void)
{
	// Pokazanie okna .
	show_mam_window(UI_fd);

	ecp_termination_notice();
}

task* return_created_ecp_task(lib::configurator &_config)
{
	return new mam(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
