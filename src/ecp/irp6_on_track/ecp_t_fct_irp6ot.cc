#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#if defined(__QNXNTO__)
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#endif /* __QNXNTO__ */

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_fct_irp6ot.h"
// Czujnik sily.
#include "ecp_mp/ecp_mp_s_force.h"
// Generator ruchu.
#include "ecp/irp6_on_track/ecp_g_fctg.h"

#include "ecp/common/ECP_main_error.h"

extern ecp_task_fct_irp6ot *ecp_t;

// Kanal komunikacyjny z procesem MP.
extern name_attach_t *ecp_attach;
// Kanal komunikacyjny z procesem UI.
name_attach_t * UI_ECP_attach;

// Obiekt generator trajektorii.
force_controlled_trajectory_generator *fctg;

// Flaga uzywana do informmowania o koncu pracy.
short TERMINATE=false;
// Flaga informujaca o zmianie trybu pracy.
bool CHANGE_CONTROL = false;
// Flaga do kalibracji czujnika.
bool CALIBRATE_SENSOR = false;
// Polecenie ruchu robota.
short MOVE_TYPE = 0;
// Rodzaj sterowania.
POSE_SPECIFICATION ps;

// Zmienna uzywana do konczenia pracy watkow.
void* value_ptr;

// Funkcja - obsluga przychodzacych sygnalow.
void ecp_task_fct_irp6ot::catch_signal(int sig)
{
	switch (sig) {
		case SIGTERM:
			// Zakonczenie pracy watkow.
			TERMINATE = true;
			// Koniec pracy czujnika.
			if (sensor_m.count(SENSOR_FORCE_ON_TRACK)>0)
				sensor_m[SENSOR_FORCE_ON_TRACK]->terminate();
			// Zwolnienie pamieci - czujnik.
			delete(sensor_m[SENSOR_FORCE_ON_TRACK]);
			// Zwolnienie pamieci - generator.
			delete(fctg);
			// Zwolnienie pamieci - robot.
			delete(ecp_m_robot);
			// Odlaczenie nazwy.
			name_detach(UI_ECP_attach, 0);
			sr_ecp_msg->message("ECP terminated");
			exit(EXIT_SUCCESS);
			break;
	}
}

/************************ UI COMMUNICATION THREAD ***************************/

// Cialo watku, ktorego zadaniem jest komunikacja z UI.
void* UI_communication_thread(void* arg)
{
	//printf("UI_communication_thread!\n");
	// Wiadomosc otrzymana z UI.
	UI_ECP_message ui_msg;
	// Wiadomosc wysylana do UI.
	ECP_message to_ui_msg;

	// Id nadawcy wiadomosci.
	int rcvid;
	// Jezeli nie przyszedl rozkaz zakonczenia.
	while (!TERMINATE) {
		// oczekiwanie na wiadomosc - wcisniety przycisk
		rcvid = MsgReceive(UI_ECP_attach->chid, &ui_msg, sizeof(ui_msg), NULL);
		// Jesli wiadomosc niewiadomego pochodzenia.
		if (rcvid == -1) {
			perror("UI_communication_thread: Receive failed\n");
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
			case FC_ADD_MACROSTEP:
				// Dodanie pozycji do trajektorii.
				fctg->add_step(ui_msg.motion_time);
				break;
			case FC_CALIBRATE_SENSOR:
				CALIBRATE_SENSOR = true;
				break;
			case FC_CHANGE_CONTROL:
				// Zmiana rodzju sterowania.
				ps = ui_msg.ps;
				CHANGE_CONTROL = true;
				break;
			case FC_MOVE_ROBOT:
				// Przepisanie rodzaju ruchu do zmiennej globalnej.
				//printf("nadeszlo %d\n", ui_msg.move_type);
				MOVE_TYPE = ui_msg.move_type;
				break;
			case FC_SAVE_TRAJECTORY:
				fctg->save_trajectory(ui_msg.filename);
				break;
			case FC_NEW_TRAJECTORY:
				// Usuniecie listy pozycji, o ile istnieje
				fctg->flush_pose_list();
				break;
			case FC_EXIT:
				// Zakonczenie ruchu.
				TERMINATE=true;
				ecp_t->ecp_termination_notice();
				break;
		}
		// Jesli trzeba odswiezyc okno.
		if (ui_msg.command == FC_GET_DATA) {
			// Pobranie polozenia.
			fctg->return_position(to_ui_msg.RS.robot_position);
			// Jesli uzywany jest czujnik sily.
			if (ecp_t->sensor_m.count(SENSOR_FORCE_ON_TRACK)>0) {
				// Przepisanie odczytow czujnika.
				fctg->return_sensor_reading(*((ecp_mp_force_sensor *)(ecp_t->sensor_m[SENSOR_FORCE_ON_TRACK])), to_ui_msg.RS.sensor_reading);
			} else {
				// Zerowe odczyty.
				for (int i =0; i<6; i++)
					to_ui_msg.RS.sensor_reading[i]=0;
			}
			// Odeslanie wiadomosci do UI.
			MsgReply(rcvid, EOK, &to_ui_msg, sizeof(ECP_message));
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
				pthread_exit(value_ptr);
			// Jezeli przyszedl rozkaz kalibracji czujnika.
			if (CALIBRATE_SENSOR) {
				// Kalibracja czujnika.
				if (ecp_t->sensor_m.count(SENSOR_FORCE_ON_TRACK)>0)
					ecp_t->sensor_m[SENSOR_FORCE_ON_TRACK]->configure_sensor();
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
		catch (ecp_generator::ECP_error e) {
			// Komunikat o bledzie wysylamy do SR.
			ecp_t->sr_ecp_msg->message (e.error_class, e.error_no);
		}
		// Pobranie polozenia robota.
		fctg->get_current_position();
		// Ruch wykonano.
		MOVE_TYPE = 0;
	}
	// Koniec dzialania.
	pthread_exit(value_ptr);
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
	ECP_message ecp_msg;
	// Odpowiedz UI do ECP.
	UI_reply ui_rep;
	// Nazwa okna (polecenie otwarcia).
	ecp_msg.hdr.type=0;
	ecp_msg.ecp_message = OPEN_FORCE_SENSOR_MOVE_WINDOW;
	// Wyslanie polecenia do UI -> otwarcie okna.
#if !defined(USE_MESSIP_SRR)
	if (MsgSend(UI_fd, &ecp_msg, sizeof(ECP_message), &ui_rep, sizeof(UI_reply)) < 0) {
#else
	int status;
	if (messip_send(UI_fd, 0,0, &ecp_msg, sizeof(ECP_message), &status, &ui_rep, sizeof(UI_reply)) < 0) {
#endif
		ecp_t->sr_ecp_msg->message(SYSTEM_ERROR, errno, "ECP: Send() to UI failed");
		throw ECP_main_error(SYSTEM_ERROR, 0);
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
ecp_task_fct_irp6ot::ecp_task_fct_irp6ot(configurator &_config) :
	ecp_task(_config)
{
}

// methods for ECP template to redefine in concrete classes
void ecp_task_fct_irp6ot::task_initialization(void)
{
	// Nawiazanie komunikacji z EDP.
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);

	// Stworzenie nazwy.
	char *tmp_name;
	tmp_name = config.return_attach_point_name	(configurator::CONFIG_SERVER, "ecp_sec_chan_attach_point", "[ecp_irp6_on_track]");

	// Dolaczenie globalnej nazwy procesu ECP - kanal do odbioru polecen z UI.
	if ((UI_ECP_attach = name_attach(NULL, tmp_name, NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
//		printf("%s\n", tmp_name);
		// W razie niepowodzenia.
		throw ECP_main_error(SYSTEM_ERROR, NAME_ATTACH_ERROR);
	}
	// Stworzenie generatora trajektorii.
	fctg = new force_controlled_trajectory_generator(*this);
	// Sprawdzanie, czy nalezy uzywac czujnik sily.
	short use_force_sensor = config.return_int_value("use_force_sensor");
	if (use_force_sensor == 1) {
		sr_ecp_msg->message("Using force sensor for move control");
		// Stworzenie obiektu czujnik.
		// ini_con->create_vsp ("[vsp_fs]");
		sensor_m[SENSOR_FORCE_ON_TRACK] = new ecp_mp_force_sensor(SENSOR_FORCE_ON_TRACK, "[vsp_fs]", *this);
		// Konfiguracja czujnika.
		sensor_m[SENSOR_FORCE_ON_TRACK]->configure_sensor();
		// Stworzenie listy czujnikow -> glowa = (czujnik sily).
		fctg->sensor_m[SENSOR_FORCE_ON_TRACK] = sensor_m[SENSOR_FORCE_ON_TRACK];
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

void ecp_task_fct_irp6ot::main_task_algorithm(void)
{
	// Pokazanie okna i uruchomienie watkow.
	show_force_control_window(UI_fd);
}

ecp_task* return_created_ecp_task(configurator &_config)
{
	return new ecp_task_fct_irp6ot(_config);
}
