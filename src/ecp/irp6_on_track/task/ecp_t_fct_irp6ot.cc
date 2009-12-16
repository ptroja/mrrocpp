#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#if defined(__QNXNTO__)
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#endif /* __QNXNTO__ */

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"
#include "ecp_mp/sensor/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/task/ecp_t_fct_irp6ot.h"
// Czujnik sily.
#include "ecp_mp/sensor/ecp_mp_s_force.h"
// Generator ruchu.
#include "ecp/irp6_on_track/generator/ecp_g_fctg.h"

#include "ecp/common/ECP_main_error.h"

#if defined(USE_MESSIP_SRR)
#include "lib/messip/messip_dataport.h"
#endif

namespace mrrocpp {
namespace ecp {
namespace common {


extern irp6ot::task::fct *ecp_t;
} // namespace common
namespace irp6ot {
namespace task {


// Kanal komunikacyjny z procesem MP.
extern name_attach_t *ecp_attach;
// Kanal komunikacyjny z procesem UI.
name_attach_t * UI_ECP_attach;

// Obiekt generator trajektorii.
generator::force_controlled_trajectory *fctg;

// Flaga uzywana do informmowania o koncu pracy.
short TERMINATE=false;
// Flaga informujaca o zmianie trybu pracy.
bool CHANGE_CONTROL = false;
// Flaga do kalibracji czujnika.
bool CALIBRATE_SENSOR = false;
// Polecenie ruchu robota.
short MOVE_TYPE = 0;
// Rodzaj sterowania.
lib::POSE_SPECIFICATION ps;

/************************ UI COMMUNICATION THREAD ***************************/

// Cialo watku, ktorego zadaniem jest komunikacja z UI.
void* UI_communication_thread(void* arg)
{
	//printf("UI_communication_thread!\n");
	// Wiadomosc otrzymana z UI.
	lib::UI_ECP_message ui_msg;
	// Wiadomosc wysylana do UI.
	lib::ECP_message to_ui_msg;

	// Id nadawcy wiadomosci.
	int rcvid;
	// Jezeli nie przyszedl rozkaz zakonczenia.
	while (!TERMINATE) {
		// oczekiwanie na wiadomosc - wcisniety przycisk
		rcvid = MsgReceive(UI_ECP_attach->chid, &ui_msg, sizeof(ui_msg), NULL);
		// Jesli wiadomosc niewiadomego pochodzenia.
		if (rcvid == -1) {
			perror("UI_communication_thread: Receive failed");
			continue;
		}
		// Jesli nadszedl puls.
		if (rcvid == 0) {
			switch (ui_msg.hdr.code) {
				case _PULSE_CODE_DISCONNECT:
					ConnectDetach(ui_msg.hdr.scoid);
				default:
					break;
			}
			continue;
		}
		// Wiadomosci z QNX IO.
		if (ui_msg.hdr.type >= _IO_BASE && ui_msg.hdr.type <= _IO_MAX) {
			MsgReply(rcvid, EOK, 0, 0);
			continue;
		}
		//printf("Zwykla wiadomosc!\n");
		// Zwykla wiadomosc.
		switch (ui_msg.command) {
			case lib::FC_ADD_MACROSTEP:
				// Dodanie pozycji do trajektorii.
				fctg->add_step(ui_msg.motion_time);
				break;
			case lib::FC_CALIBRATE_SENSOR:
				CALIBRATE_SENSOR = true;
				break;
			case lib::FC_CHANGE_CONTROL:
				// Zmiana rodzju sterowania.
				ps = ui_msg.ps;
				CHANGE_CONTROL = true;
				break;
			case lib::FC_MOVE_ROBOT:
				// Przepisanie rodzaju ruchu do zmiennej globalnej.
				//printf("nadeszlo %d\n", ui_msg.move_type);
				MOVE_TYPE = ui_msg.move_type;
				break;
			case lib::FC_SAVE_TRAJECTORY:
				fctg->save_trajectory(ui_msg.filename);
				break;
			case lib::FC_NEW_TRAJECTORY:
				// Usuniecie listy pozycji, o ile istnieje
				fctg->flush_pose_list();
				break;
			case lib::FC_EXIT:
				// Zakonczenie ruchu.
				TERMINATE=true;
				common::ecp_t->ecp_termination_notice();
				break;
			default:
				fprintf(stderr, "unknown UI command in %s:%d\n", __FILE__, __LINE__);
				break;
		}
		// Jesli trzeba odswiezyc okno.
		if (ui_msg.command == lib::FC_GET_DATA) {
			// Pobranie polozenia.
			fctg->return_position(to_ui_msg.RS.robot_position);
			// Jesli uzywany jest czujnik sily.
			if (common::ecp_t->sensor_m.count(lib::SENSOR_FORCE_ON_TRACK)>0) {
				// Przepisanie odczytow czujnika.
				fctg->return_sensor_reading(*((ecp_mp::sensor::force *)(common::ecp_t->sensor_m[lib::SENSOR_FORCE_ON_TRACK])), to_ui_msg.RS.sensor_reading);
			} else {
				// Zerowe odczyty.
				for (int i =0; i<6; i++)
					to_ui_msg.RS.sensor_reading[i]=0;
			}
			// Odeslanie wiadomosci do UI.
			MsgReply(rcvid, EOK, &to_ui_msg, sizeof(lib::ECP_message));
		} else {
			// Wyslanie standardowej odpowiedzi.
			MsgReply(rcvid, EOK, NULL, 0);
		}
	}
	return NULL;
}

/*********************** FORCE SENSOR MOVE THREAD *************************/

// Cialo watku, ktorego zadaniem jest poruszanie robotem.
void* forcesensor_move_thread(void* arg)
{
	// Pobranie polozenia robota.
	fctg->get_current_position();
	// Jezeli nie przyszedl rozkaz zakonczenia.
	while (!TERMINATE) {
		// Oczekiwanie na polecenie ruchu.
		do {
			usleep(1000*50);
			// Jezeli koniec pracy.
			if (TERMINATE)
				return NULL;
			// Jezeli przyszedl rozkaz kalibracji czujnika.
			if (CALIBRATE_SENSOR) {
				// Kalibracja czujnika.
				if (common::ecp_t->sensor_m.count(lib::SENSOR_FORCE_ON_TRACK)>0)
					common::ecp_t->sensor_m[lib::SENSOR_FORCE_ON_TRACK]->configure_sensor();
				CALIBRATE_SENSOR = false;
			}
			// Jezeli przyszedl rozkaz zmiany sterowania.
			if (CHANGE_CONTROL) {
				// Zmiana trybu.
				fctg->change_control(ps);
				CHANGE_CONTROL = false;
				// Pobranie aktualnych pozycji.
				fctg->get_current_position();
			}
		} while (MOVE_TYPE == 0);
		// Ustawienie rodzaju ruchu.
		fctg->set_move_type(MOVE_TYPE);
		try {
			// Proba wykonania ruchu w danym kierunku.
			fctg->Move();
		}
		catch (common::generator::generator::ECP_error e) {
			// Komunikat o bledzie wysylamy do SR.
			common::ecp_t->sr_ecp_msg->message (e.error_class, e.error_no);
		}
		// Pobranie polozenia robota.
		fctg->get_current_position();
		// Ruch wykonano.
		MOVE_TYPE = 0;
	}

	// Koniec dzialania.
	return NULL;
}

/********************** SHOW FORCE CONTROL WINDOW ************************/

// Funkcja pokazuje okno oraz odpala watki.
void show_force_control_window
#if !defined(USE_MESSIP_SRR)
(int UI_fd)
#else
(messip_channel_t *UI_fd)
#endif
{
	int i;
	// Przesylka z ECP do UI.
	lib::ECP_message ecp_msg;
	// Odpowiedz UI do ECP.
	lib::UI_reply ui_rep;
	// Nazwa okna (polecenie otwarcia).
	ecp_msg.hdr.type=0;
	ecp_msg.ecp_message = lib::OPEN_FORCE_SENSOR_MOVE_WINDOW;
	// Wyslanie polecenia do UI -> otwarcie okna.
#if !defined(USE_MESSIP_SRR)
	if (MsgSend(UI_fd, &ecp_msg, sizeof(lib::ECP_message), &ui_rep, sizeof(lib::UI_reply)) < 0) {
#else
	if (messip::port_send(UI_fd, 0, 0, ecp_msg, ui_rep) < 0) {
#endif
		common::ecp_t->sr_ecp_msg->message(lib::SYSTEM_ERROR, errno, "ECP: Send() to UI failed");
		throw common::ECP_main_error(lib::SYSTEM_ERROR, 0);
	}
	// Ustawienie flagi konczenia pracy.
	TERMINATE = false;
	// Atrybuty watku.
	pthread_t tid;
	pthread_attr_t tattr;
	pthread_attr_init( &tattr);
	pthread_attr_setdetachstate( &tattr, PTHREAD_CREATE_DETACHED);
	// Odpalenie watku poruszajacego robotem.
	pthread_create(&tid, &tattr, &forcesensor_move_thread, (void *)i);
	// Odpalenie watku komunikacji z UI.
	UI_communication_thread((void *)i);
}

// KONSTRUKTORY
fct::fct(lib::configurator &_config) :
	task(_config)
{
	// Nawiazanie komunikacji z EDP.
	ecp_m_robot = new robot (*this);

	// Stworzenie nazwy.
	std::string attach_point = config.return_attach_point_name	(lib::configurator::CONFIG_SERVER, "ecp_sec_chan_attach_point", ECP_IRP6_ON_TRACK_SECTION);

	// Dolaczenie globalnej nazwy procesu ECP - kanal do odbioru polecen z UI.
	if ((UI_ECP_attach = name_attach(NULL, attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
//		printf("%s\n", tmp_name);
		// W razie niepowodzenia.
		throw common::ECP_main_error(lib::SYSTEM_ERROR, NAME_ATTACH_ERROR);
	}
	// Stworzenie generatora trajektorii.
	fctg = new generator::force_controlled_trajectory(*this);
	// Sprawdzanie, czy nalezy uzywac czujnik sily.
	short use_force_sensor = config.return_int_value("use_force_sensor");
	if (use_force_sensor == 1) {
		sr_ecp_msg->message("Using force sensor for move control");
		// Stworzenie obiektu czujnik.
		// ini_con->create_vsp ("[vsp_fs]");
		sensor_m[lib::SENSOR_FORCE_ON_TRACK] = new ecp_mp::sensor::force(lib::SENSOR_FORCE_ON_TRACK, "[vsp_fs]", *this);
		// Konfiguracja czujnika.
		sensor_m[lib::SENSOR_FORCE_ON_TRACK]->configure_sensor();
		// Stworzenie listy czujnikow -> glowa = (czujnik sily).
		fctg->sensor_m[lib::SENSOR_FORCE_ON_TRACK] = sensor_m[lib::SENSOR_FORCE_ON_TRACK];
		// Odczyt wielkosci niebezpiecznej sily z pliku INI.
		fctg->set_dangerous_force();
	} else {
		sr_ecp_msg->message("Not using force sensor for move control");
		// Pusta lista czujnikow.
		fctg->sensor_m.clear();
		sensor_m.clear();
		// Pusty czujnik.

	}
	sr_ecp_msg->message("ECP loaded");
}

void fct::main_task_algorithm(void)
{
	// Pokazanie okna i uruchomienie watkow.
	show_force_control_window(UI_fd);
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new irp6ot::task::fct(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

