#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <stdlib.h>
#if !defined(USE_MESSIP_SRR)
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#else
#include "lib/messip/messip_dataport.h"
#endif

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/common/ecp_t_mam.h"

// Czujnik.
#include "ecp_mp/ecp_mp_s_digital_scales.h"

#include "ecp/common/ECP_main_error.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace task {

/************************ UI COMMUNICATION THREAD ***************************/
void* mam::UI_communication_thread(void* arg)
{
	mam & mam_task = * (mam *) arg;

	while (!mam_task.TERMINATE) {
		// Wiadomosc otrzymana z UI.
		lib::UI_ECP_message from_ui_msg;
#if !defined(USE_MESSIP_SRR)
		// Oczekiwanie na wiadomosc (wcisniety przycisk).
		int rcvid = MsgReceive(UI_ECP_attach->chid, &from_ui_msg, sizeof(from_ui_msg), NULL);
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
		// Wiadomosci z QNX IO.
		if (from_ui_msg.hdr.type >= _IO_BASE && from_ui_msg.hdr.type <= _IO_MAX) {
			MsgReply(rcvid, EOK, 0, 0);
			continue;
		}
#else
		int type, subtype;
		int rcvid = messip::port_receive(UI_ECP_attach, type, subtype, from_ui_msg);
		if (rcvid < 0) {
			fprintf(stderr, "MAM: messip::port_receive() failed\n");
			continue;
		}
#endif
		// Zwykla wiadomosc.
		switch (from_ui_msg.command) {
			case lib::MAM_START:
				// Rozpoczecie wykonywania pomiarow.
				mam_task.START_MEASURES = true;
				break;
			case lib::MAM_STOP:
				// Zakonczenie wykonywania pomiarow.
				mam_task.START_MEASURES = false;
				break;
			case lib::MAM_SAVE:
				// Zapis do pliku.
				mam_task.mam_gen->save_mam_list(from_ui_msg.filename);
				break;
			case lib::MAM_CLEAR:
				// Oproznienie listy z pomiarami.
				mam_task.mam_gen->flush_mam_list();
				break;
			case lib::MAM_CALIBRATE:
				// Konfiguracja czujnika.
				mam_task.sensor_m[lib::SENSOR_DIGITAL_SCALE_SENSOR]->configure_sensor();
				break;
			case lib::MAM_EXIT:
				// Zakonczenie dzialania procesu.
				mam_task.TERMINATE = true;
				break;
			default:
				fprintf(stderr, "unknown MAM command in %s:%d\n", __FILE__, __LINE__);
				break;
		}

		// Odeslanie pustej odpowiedzi
		// TODO: error checking
#if !defined(USE_MESSIP_SRR)
		MsgReply(rcvid, EOK, NULL, 0);
#else
		messip::port_reply_ack(UI_ECP_attach, rcvid);
#endif
	}
	return NULL;
}

/*************************** MEASURES THREAD ******************************/
void* measures_thread(void* arg)
{
	mam & mam_task = * (mam *) arg;

	// Jezeli nie przyszedl rozkaz zakonczenia.
	while (!mam_task.TERMINATE) {
		// Oczekiwanie na rozkaz wykonania pierwszego kroku.
		while (!mam_task.START_MEASURES) {
			usleep(1000*50);
			// Jezeli koniec pracy.
			if (mam_task.TERMINATE)
				return NULL;
		}
		// Zebranie pomiarow co np. 300 ms.
		mam_task.mam_gen->Move();
		// Oraz cale porownanie.
		usleep(1000*300);
		// Oraz odswierzenie okna.
		//        ((ecp_task_mam*)ecp_t)->sr_ecp_msg->message("Tak sobie iteruje.");
	}

	// koniec dzialania
	return NULL;
}

/*************************** SHOW MAM WINDOW *****************************/
void mam::show_mam_window
#if !defined(USE_MESSIP_SRR)
	(int UI_fd)
#else
	(messip_channel_t *UI_fd)
#endif
{
	lib::ECP_message ecp_ui_msg; // Przesylka z ECP do UI

	// Nazwa okna (polecenie otwarcia).
	ecp_ui_msg.ecp_message = lib::MAM_OPEN_WINDOW;

	// Wyslanie polecenia do UI -> otwarcie okna
#if !defined(USE_MESSIP_SRR)
	ecp_ui_msg.hdr.type=0;
	if (MsgSend(UI_fd, &ecp_ui_msg, sizeof(lib::ECP_message), NULL, 0) < 0)
#else
	if (messip::port_send_sync(UI_fd, 0, 0, ecp_ui_msg) < 0)
#endif
	{
		perror("show_mam_window: Send to UI failed");
		throw ECP_main_error(lib::SYSTEM_ERROR, 0);
	}

	// Ustawienie flagi konczenia pracy.
	TERMINATE = false;
	// Ustawienie flagi zbierania pomiarow.
	START_MEASURES=false;

	// Odpalenie watku poruszajacego robotem
	pthread_t tid;
	pthread_create(&tid, NULL, &measures_thread, NULL);

	// Odpalenie watku komunikacji z UI.
	UI_communication_thread(NULL);

	// Oczekiwanie na zakonczenie watku poruszajacego robotem
	int join_result = pthread_join(tid, NULL);
	if (join_result != 0) {
		fprintf(stderr, "pthread_join() failed: %s\n", strerror(join_result));
	}
}

// KONSTRUKTORY
mam::mam(lib::configurator &_config) :
	task(_config)
{
	// the robot is choose dependendant on the section of configuration file sent as argv[4]
	if (config.section_name == ECP_IRP6_ON_TRACK_SECTION) {
		ecp_m_robot = new irp6ot::robot (*this);
	} else if (config.section_name == ECP_IRP6_POSTUMENT_SECTION) {
		ecp_m_robot = new irp6p::robot (*this);
	}

	// Dolaczenie globalnej nazwy procesu ECP - kanal do odbioru polecen z UI.
#if !defined(USE_MESSIP_SRR)
	if ((UI_ECP_attach = name_attach(NULL, "ECP_M_MAM", NAME_FLAG_ATTACH_GLOBAL)) == NULL)
#else
	if ((UI_ECP_attach = messip::port_create(NULL, "ECP_M_MAM")) == NULL)
#endif
	{
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
	// Pokazanie okna
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
