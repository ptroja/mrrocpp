#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <strings.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <dirent.h>

#include <QtGui/QApplication>
#include "mainwindow.h"

#include "interface.h"
#include "ui_sr.h"
#include "spkm/ui_r_spkm.h"

namespace mrrocpp {
namespace ui {
namespace common {

Interface::Interface() :
	config(NULL), all_ecp_msg(NULL), ui_msg(NULL), is_mp_and_ecps_active(false), all_edps(UI_ALL_EDPS_NONE_EDP_LOADED)
{

	mw = new MainWindow(*this);

	main_eb = new function_execution_buffer(*this);

	mp.state = UI_MP_NOT_PERMITED_TO_RUN;// mp wylaczone
	mp.last_state = UI_MP_NOT_PERMITED_TO_RUN;// mp wylaczone
	mp.pid = -1;
	ui_state = 1;// ui working
	file_window_mode = ui::common::FSTRAJECTORY; // uczenie
	is_task_window_open = false;// informacja czy okno zadanai jest otwarte
	is_process_control_window_open = false;// informacja czy okno sterowania procesami jest otwarte
	process_control_window_renew = true;
	is_file_selection_window_open = false;
	is_teaching_window_open = false;
	mrrocpp_bin_to_root_path = "../../";

}

int Interface::set_ui_state_notification(UI_NOTIFICATION_STATE_ENUM new_notifacion)
{
	if (new_notifacion != notification_state) {

		notification_state = new_notifacion;

		switch (new_notifacion)
		{
			case UI_N_STARTING:
				mw->ui_notification("STARTING", Qt::magenta);

				break;
			case UI_N_READY:
				mw->ui_notification("READY", Qt::blue);

				break;
			case UI_N_BUSY:
				mw->ui_notification("BUSY", Qt::red);

				break;
			case UI_N_EXITING:
				mw->ui_notification("EXITING", Qt::magenta);

				break;
			case UI_N_COMMUNICATION:
				mw->ui_notification("COMMUNICATION", Qt::red);

				break;
			case UI_N_SYNCHRONISATION:
				mw->ui_notification("SYNCHRONISATION", Qt::red);

				break;
			case UI_N_PROCESS_CREATION:
				mw->ui_notification("PROCESS CREATION", Qt::red);

				break;
		}

		return 1;

	}

	return 0;

}

void Interface::init()
{

	// ustalenie katalogow UI

	mw->show();

	struct utsname sysinfo;
	char* cwd;
	char buff[PATH_MAX + 1];

	if (uname(&sysinfo) == -1) {
		perror("uname");
	}

	cwd = getcwd(buff, PATH_MAX + 1);
	if (cwd == NULL) {
		perror("Blad cwd w UI");
	}

	spkm = new spkm::UiRobot(*this);
	robot_m[spkm->robot_name] = spkm;

	/* TR
	 smb = new smb::UiRobot(*this);
	 robot_m[smb->robot_name] = smb;

	 shead = new shead::UiRobot(*this);
	 robot_m[shead->robot_name] = shead;

	 bird_hand = new bird_hand::UiRobot(*this);
	 robot_m[bird_hand->robot_name] = bird_hand;

	 irp6ot_m = new irp6ot_m::UiRobot(*this);
	 robot_m[irp6ot_m->robot_name] = irp6ot_m;

	 irp6ot_tfg = new irp6ot_tfg::UiRobot(*this);
	 robot_m[irp6ot_tfg->robot_name] = irp6ot_tfg;

	 irp6p_m = new irp6p_m::UiRobot(*this);
	 robot_m[irp6p_m->robot_name] = irp6p_m;

	 irp6p_tfg = new irp6p_tfg::UiRobot(*this);
	 robot_m[irp6p_tfg->robot_name] = irp6p_tfg;

	 sarkofag = new sarkofag::UiRobot(*this);
	 robot_m[sarkofag->robot_name] = sarkofag;

	 irp6m_m = new irp6m::UiRobot(*this);
	 robot_m[irp6m_m->robot_name] = irp6m_m;

	 conveyor = new conveyor::UiRobot(*this);
	 robot_m[conveyor->robot_name] = conveyor;

	 speaker = new speaker::UiRobot(*this);
	 robot_m[speaker->robot_name] = speaker;

	 polycrank = new polycrank::UiRobot(*this);
	 robot_m[polycrank->robot_name] = polycrank;
	 */
	ui_node_name = sysinfo.nodename;
	is_sr_thread_loaded = false;

	binaries_local_path = cwd;
	mrrocpp_local_path = cwd;
	mrrocpp_local_path.erase(mrrocpp_local_path.length() - 3);// kopiowanie lokalnej sciezki bez "bin" - 3 znaki
	//mrrocpp_local_path += "../";
	binaries_network_path = "/net/";
	binaries_network_path += ui_node_name;
	binaries_network_path += binaries_local_path;
	binaries_network_path += "/";// wysylane jako argument do procesow potomnych (mp_m i dalej)
	// printf( "system name  : %s\n", binaries_network_path);

	// sciezka dla okna z wyborem pliku podczas wybor trajektorii dla uczenia
	teach_filesel_fullpath = "/net/";
	teach_filesel_fullpath += ui_node_name;
	teach_filesel_fullpath += mrrocpp_local_path;
	//	teach_filesel_fullpath += "trj";
	// 	printf("abba: %s\n", teach_filesel_fullpath);

	// sciezka dla okna z wyborem pliku z trajektoria podczas wyboru pliku konfiguracyjnego
	config_file_fullpath = "/net/";
	config_file_fullpath += ui_node_name;
	config_file_fullpath += mrrocpp_local_path;
	config_file_fullpath += "../";

	// printf ("Remember to create gns server\n");


	set_ui_state_notification(UI_N_STARTING);
	/*TR
	 signal(SIGINT, &catch_signal);// by y aby uniemozliwic niekontrolowane zakonczenie aplikacji ctrl-c z kalwiatury
	 signal(SIGALRM, &catch_signal);
	 signal(SIGSEGV, &catch_signal);

	 signal(SIGCHLD, &catch_signal);

	 lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 6);
	 */
	// pierwsze zczytanie pliku konfiguracyjnego (aby pobrac nazwy dla pozostalych watkow UI)
	if (get_default_configuration_file_name() >= 1) // zczytaj nazwe pliku konfiguracyjnego
	{
		std::cerr << "ui a" << std::endl;
		initiate_configuration();
		// sprawdza czy sa postawione gns's i ew. stawia je
		// uwaga serwer musi byc wczesniej postawiony
		check_gns();
	} else {
		printf("Blad manage_default_configuration_file\n");
		/* TR
		 PtExit(EXIT_SUCCESS);
		 */
	}

	create_threads();

	// Zablokowanie domyslnej obslugi sygnalu SIGINT w watkach UI_SR i UI_COMM


	// kolejne zczytanie pliku konfiguracyjnego
	if (get_default_configuration_file_name() == 1) // zczytaj nazwe pliku konfiguracyjnego
	{
		std::cerr << "ui b" << std::endl;
		reload_whole_configuration();

	} else {
		printf("Blad manage_default_configuration_file\n");
		/* TR
		 PtExit(EXIT_SUCCESS);
		 */
	}
	std::cerr << "ui c" << std::endl;
	// inicjacja pliku z logami sr
	check_gns();

	time_t time_of_day;
	char file_date[50];
	char log_file_with_dir[100];
	char file_name[50];

	time_of_day = time(NULL);
	strftime(file_date, 40, "%g%m%d_%H-%M-%S", localtime(&time_of_day));

	sprintf(file_name, "/%s_sr_log", file_date);

	// 	strcpy(file_name,"/pomiar.p");
	strcpy(log_file_with_dir, (mrrocpp_bin_to_root_path + "logs/").c_str());

	if (access(log_file_with_dir, R_OK) != 0) {
		mkdir(log_file_with_dir, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
	}

	strcat(log_file_with_dir, file_name);

	log_file_outfile = new std::ofstream(log_file_with_dir, std::ios::out);

	if (!(*log_file_outfile)) {
		std::cerr << "Cannot open file: " << file_name << '\n';
		perror("because of");
	}

	//ui_msg->message("closing");

	manage_interface();

}

int Interface::MPup_int()

{

	set_ui_state_notification(UI_N_PROCESS_CREATION);

	if (mp.pid == -1) {

		mp.node_nr = config->return_node_number(mp.node_name.c_str());

		std::string mp_network_pulse_attach_point("/dev/name/global/");
		mp_network_pulse_attach_point += mp.network_pulse_attach_point;

		// sprawdzenie czy nie jest juz zarejestrowany serwer komunikacyjny MP
		if (access(mp_network_pulse_attach_point.c_str(), R_OK) == 0) {
			ui_msg->message(lib::NON_FATAL_ERROR, "mp already exists");
		} else if (check_node_existence(mp.node_name, "mp")) {
			mp.pid = config->process_spawn(lib::MP_SECTION);

			if (mp.pid > 0) {

				short tmp = 0;
				// kilka sekund  (~1) na otworzenie urzadzenia
				while ((mp.pulse_fd =
#if !defined(USE_MESSIP_SRR)
						name_open(mp.network_pulse_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) < 0
#else
					messip::port_connect(mp.network_pulse_attach_point)) == NULL
#endif
					)
					if ((tmp++) < lib::CONNECT_RETRY)
						usleep(lib::CONNECT_DELAY);
					else {
						fprintf(stderr, "name_open() for %s failed: %s\n", mp.network_pulse_attach_point.c_str(), strerror(errno));
						break;
					}

				teachingstate = ui::common::MP_RUNNING;

				mp.state = ui::common::UI_MP_WAITING_FOR_START_PULSE; // mp wlaczone
				/* TR
				 pt_res = PtEnter(0);
				 start_process_control_window(NULL, NULL, NULL);
				 if (pt_res >= 0)
				 PtLeave(0);
				 */
			} else {
				fprintf(stderr, "mp spawn failed\n");
			}
			manage_interface();
		}
	}

	return 1;
}

// funkcja odpowiedzialna za wyglad aplikacji na podstawie jej stanu

int Interface::manage_interface(void)
{

	// UWAGA ta funkcja powinna byc odporna na odpalaenie z dowolnego watku !!!

	check_edps_state_and_modify_mp_state();
	/*TR
	 // na wstepie wylaczamy przyciski EDP z all robots menu. Sa one ewentualnie wlaczane dalej
	 ApModifyItemState(&all_robots_menu, AB_ITEM_DIM, ABN_mm_all_robots_preset_positions, ABN_mm_all_robots_synchronisation, ABN_mm_all_robots_edp_unload, ABN_mm_all_robots_edp_load, NULL);

	 // menu file
	 // ApModifyItemState( &file_menu, AB_ITEM_DIM, NULL);

	 */
	// uruchmomienie manage interface dla wszystkich robotow
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->manage_interface();
				}

	// wlasciwosci menu  ABW_base_all_robots


	switch (all_edps)
	{
		case UI_ALL_EDPS_NONE_EDP_ACTIVATED:
			/* TR
			 //				printf("UI_ALL_EDPS_NONE_EDP_ACTIVATED\n");
			 block_widget( ABW_base_all_robots);
			 PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_GRAY, 0);
			 block_widget( ABW_base_robot);
			 PtSetResource(ABW_base_robot, Pt_ARG_COLOR, Pg_GRAY, 0);
			 */
			break;
		case UI_ALL_EDPS_NONE_EDP_LOADED:
			/* TR
			 //				printf("UI_ALL_EDPS_NONE_EDP_LOADED\n");
			 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_load, NULL);
			 PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_BLACK, 0);
			 PtSetResource(ABW_base_robot, Pt_ARG_COLOR, Pg_BLACK, 0);
			 unblock_widget(ABW_base_all_robots);
			 unblock_widget(ABW_base_robot);
			 */
			break;
		case UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED:
			/* TR
			 //			printf("UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED\n");
			 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_unload, NULL);
			 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_load, NULL);
			 PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_BLACK, 0);
			 PtSetResource(ABW_base_robot, Pt_ARG_COLOR, Pg_BLACK, 0);
			 unblock_widget(ABW_base_all_robots);
			 unblock_widget(ABW_base_robot);
			 */
			break;
		case UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED:
			/* TR
			 //			printf("UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED\n");
			 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_unload, NULL);
			 PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_DBLUE, 0);
			 unblock_widget(ABW_base_all_robots);
			 unblock_widget(ABW_base_robot);
			 */
			break;
		case UI_ALL_EDPS_LOADED_AND_SYNCHRONISED:
			/* TR
			 //				printf("UI_ALL_EDPS_LOADED_AND_SYNCHRONISED\n");
			 PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_BLUE, 0);
			 unblock_widget(ABW_base_all_robots);
			 unblock_widget(ABW_base_robot);

			 // w zaleznosci od stanu MP
			 switch (mp.state)
			 {
			 case common::UI_MP_NOT_PERMITED_TO_RUN:
			 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_unload, NULL);
			 break;
			 case common::UI_MP_PERMITED_TO_RUN:
			 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_unload, ABN_mm_all_robots_preset_positions, NULL);
			 break;
			 case common::UI_MP_WAITING_FOR_START_PULSE:
			 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_preset_positions, NULL);
			 break;
			 case common::UI_MP_TASK_RUNNING:
			 case common::UI_MP_TASK_PAUSED:
			 ApModifyItemState(&all_robots_menu, AB_ITEM_DIM, ABN_mm_all_robots_preset_positions, NULL);
			 break;
			 default:
			 break;
			 }
			 */
			break;
		default:
			break;
	}

	// wlasciwosci menu task_menu
	switch (mp.state)
	{

		case common::UI_MP_NOT_PERMITED_TO_RUN:
			/* TR
			 ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_load, ABN_mm_mp_unload, NULL);
			 PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_BLACK, 0);
			 */
			break;
		case common::UI_MP_PERMITED_TO_RUN:
			/* TR
			 ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_unload, NULL);
			 ApModifyItemState(&task_menu, AB_ITEM_NORMAL, ABN_mm_mp_load, NULL);
			 PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_BLACK, 0);
			 */
			break;
		case common::UI_MP_WAITING_FOR_START_PULSE:
			/* TR
			 ApModifyItemState(&task_menu, AB_ITEM_NORMAL, ABN_mm_mp_unload, NULL);
			 ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_load, NULL);
			 //	ApModifyItemState( &all_robots_menu, AB_ITEM_DIM, ABN_mm_all_robots_edp_unload, NULL);
			 PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_DBLUE, 0);
			 */
			break;
		case common::UI_MP_TASK_RUNNING:
		case common::UI_MP_TASK_PAUSED:
			/* TR
			 ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_unload, ABN_mm_mp_load, NULL);
			 PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_BLUE, 0);
			 */
			break;
		default:
			break;
	}

	return 1;
}

void Interface::reload_whole_configuration()
{

	if (access(config_file_relativepath.c_str(), R_OK) != 0) {
		std::cerr << "Wrong entry in default_file.cfg - load another configuration than: " << config_file_relativepath
				<< std::endl;
		config_file_relativepath = mrrocpp_bin_to_root_path + "configs/common.ini";
	}

	if ((mp.state == UI_MP_NOT_PERMITED_TO_RUN) || (mp.state == UI_MP_PERMITED_TO_RUN)) { // jesli nie dziala mp podmien mp ecp vsp


#if !defined(USE_MESSIP_SRR)
		// funkcja dziala niepoprawnie z config serwerem
		config->change_config_file(config_file);
#endif

		is_mp_and_ecps_active = config->value <int> ("is_mp_and_ecps_active");

		switch (all_edps)
		{
			case UI_ALL_EDPS_NONE_EDP_ACTIVATED:
			case UI_ALL_EDPS_NONE_EDP_LOADED:

				// uruchmomienie manage interface dla wszystkich robotow
				BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
							{
								robot_node.second->reload_configuration();
							}

				break;
			default:
				break;
		}

		// clearing of lists
		clear_all_configuration_lists();

		// sczytanie listy sekcji
		fill_section_list(config_file_relativepath.c_str());
		fill_section_list((mrrocpp_bin_to_root_path + "configs/common.ini").c_str());
		fill_node_list();
		fill_program_node_list();

		/*
		 for (list<char*>::iterator list_iterator = section_list.begin(); list_iterator != section_list.end(); list_iterator++)
		 {
		 printf("section_name: %s\n", *list_iterator);

		 }

		 for (list<char*>::iterator node_list_iterator = ui_state.node_list.begin(); node_list_iterator != ui_state.node_list.end(); node_list_iterator++)
		 {
		 printf("node_name: %s\n", *node_list_iterator);
		 }

		 for (list<program_node_def>::iterator program_node_list_iterator = program_node_list.begin(); program_node_list_iterator != program_node_list.end(); program_node_list_iterator++)
		 {
		 printf("node_name: %s\n", program_node_list_iterator->node_name);
		 }
		 */

		// zczytanie konfiguracji UI


		// zczytanie konfiguracji MP

		if (is_mp_and_ecps_active) {
			mp.network_pulse_attach_point
					= config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "mp_pulse_attach_point", lib::MP_SECTION);
			mp.node_name = config->value <std::string> ("node_name", lib::MP_SECTION);
			mp.pid = -1;
		}

		// inicjacja komunikacji z watkiem sr
		if (ui_msg == NULL) {
			ui_msg = new lib::sr_ui(lib::UI, ui_attach_point.c_str(), network_sr_attach_point.c_str(), false);
		}

		// inicjacja komunikacji z watkiem sr
		if (all_ecp_msg == NULL) {
			all_ecp_msg = new lib::sr_ecp(lib::ECP, "ui_all_ecp", network_sr_attach_point.c_str(), false);
		}

		// wypisanie komunikatu o odczytaniu konfiguracji
		if (ui_msg) {
			std::string msg(config_file);
			msg += " config file loaded";
			ui_msg->message(msg.c_str());
		}

	}

	manage_interface();
}

void Interface::UI_close(void)
{
	printf("UI CLOSING\n");
	delay(100);// czas na ustabilizowanie sie edp
	ui_state = 2;// funcja OnTimer dowie sie ze aplikacja ma byc zamknieta
}

void Interface::abort_threads()

{
#if defined(__QNXNTO__)
	delete ui_sr_obj;
	delete ui_ecp_obj;

	delete meb_tid;
#endif
}

bool Interface::check_node_existence(const std::string & _node, const std::string & beginnig_of_message)
{

	return true;
}

// sprawdza czy sa postawione gns's i ew. stawia je
// uwaga serwer powinien byc wczesniej postawiony (dokladnie jeden w sieci)
// jesli dziala messip (np. pod linux) nieaktywne

int Interface::check_gns()
{

	return 1;
}

bool Interface::is_any_robot_active()
{
	bool r_value = false;

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if (robot_node.second->state.is_active) {
						return true;
					}
				}

	return r_value;
}

bool Interface::are_all_robots_synchronised_or_inactive()
{
	bool r_value = true;

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					r_value = r_value && (((robot_node.second->state.is_active)
							&& (robot_node.second->state.edp.is_synchronised))
							|| (!(robot_node.second->state.is_active)));

					if (!r_value) {
						return false;
					}
				}

	return r_value;
}

bool Interface::are_all_robots_loaded_or_inactive()
{
	bool r_value = true;

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					r_value = r_value && (((robot_node.second->state.is_active) && (robot_node.second->state.edp.state
							> 0)) || (!(robot_node.second->state.is_active)));

					if (!r_value) {
						return false;
					}
				}

	return r_value;
}

bool Interface::is_any_active_robot_loaded()
{
	bool r_value = false;

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if ((robot_node.second->state.is_active) && (robot_node.second->state.edp.state > 0)) {
						return true;
					}
				}

	return r_value;
}

// ustala stan wszytkich EDP
int Interface::check_edps_state_and_modify_mp_state()
{

	// wyznaczenie stanu wszytkich EDP abstahujac od MP

	// jesli wszytkie sa nieaktywne
	if (!is_any_robot_active()) {
		all_edps = UI_ALL_EDPS_NONE_EDP_ACTIVATED;

		// jesli wszystkie sa zsynchronizowane
	} else if (are_all_robots_synchronised_or_inactive()) {
		all_edps = UI_ALL_EDPS_LOADED_AND_SYNCHRONISED;

		// jesli wszystkie sa zaladowane
	} else if (are_all_robots_loaded_or_inactive()) {
		all_edps = UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED;

		// jesli chociaz jeden jest zaladowany
	} else if (is_any_active_robot_loaded()) {
		all_edps = UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED;

		// jesli zaden nie jest zaladowany
	} else {
		all_edps = UI_ALL_EDPS_NONE_EDP_LOADED;

	}

	// modyfikacja stanu MP przez stan wysztkich EDP

	switch (all_edps)
	{
		case UI_ALL_EDPS_NONE_EDP_ACTIVATED:
		case UI_ALL_EDPS_LOADED_AND_SYNCHRONISED:
			if ((mp.state == UI_MP_NOT_PERMITED_TO_RUN) && (is_mp_and_ecps_active)) {
				mp.state = UI_MP_PERMITED_TO_RUN; // pozwol na uruchomienie mp
			}
			break;

		case UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED:
		case UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED:
		case UI_ALL_EDPS_NONE_EDP_LOADED:
			if (mp.state == UI_MP_PERMITED_TO_RUN) {
				mp.state = UI_MP_NOT_PERMITED_TO_RUN; // nie pozwol na uruchomienie mp
			}
			break;
		default:
			break;
	}
	return 1;
}

// odczytuje nazwe domyslengo pliku konfiguracyjnego, w razie braku ustawia common.ini
int Interface::get_default_configuration_file_name()
{

	FILE * fp = fopen((mrrocpp_bin_to_root_path + "configs/default_file.cfg").c_str(), "r");
	if (fp != NULL) {
		//printf("alala\n");
		char tmp_buf[255];
		fgets(tmp_buf, 255, fp); // Uwaga na zwracanego NULLa
		char *tmp_buf1 = strtok(tmp_buf, "=\n\r"); // get first token
		config_file = tmp_buf1;

		config_file_relativepath = mrrocpp_bin_to_root_path;
		config_file_relativepath += config_file;

		fclose(fp);
		return 1;

	} else {
		//	printf("balala\n");
		// jesli plik z domyslna konfiguracja (default_file.cfg) nie istnieje to utworz go i wpisz do niego common.ini
		printf("Utworzono plik default_file.cfg z konfiguracja common.ini\n");
		fp = fopen((mrrocpp_bin_to_root_path + "configs/default_file.cfg").c_str(), "w");
		fclose(fp);

		config_file = "configs/common.ini";
		config_file_relativepath = mrrocpp_bin_to_root_path;
		config_file_relativepath += config_file;

		std::ofstream outfile((mrrocpp_bin_to_root_path + "configs/default_file.cfg").c_str(), std::ios::out);
		if (!outfile.good()) {
			std::cerr << "Cannot open file: default_file.cfg" << std::endl;
			perror("because of");
		} else
			outfile << config_file;

		return 2;
	}
}

// zapisuje nazwe domyslengo pliku konfiguracyjnego
int Interface::set_default_configuration_file_name()
{

	config_file_relativepath = mrrocpp_bin_to_root_path;
	config_file_relativepath += config_file;

	std::ofstream outfile((mrrocpp_bin_to_root_path + "configs/default_file.cfg").c_str(), std::ios::out);
	if (!outfile.good()) {
		std::cerr << "Cannot open file: default_file.cfg\n";
		perror("because of");
	} else
		outfile << config_file;

	return 1;
}

// fills program_node list
int Interface::fill_program_node_list()
{
	//	printf("fill_program_node_list\n");

	for (std::list <Interface::list_t>::iterator section_list_iterator = section_list.begin(); section_list_iterator
			!= section_list.end(); section_list_iterator++) {

		if ((config->exists("program_name", *section_list_iterator)
				&& config->exists("node_name", *section_list_iterator))) {
			//	char* tmp_p =config->value<std::string>("program_name", *section_list_iterator);
			//	char* tmp_n =config->value<std::string>("node_name", *section_list_iterator);

			program_node_def tmp_s;

			tmp_s.program_name = config->value <std::string> ("program_name", *section_list_iterator);
			tmp_s.node_name = config->value <std::string> ("node_name", *section_list_iterator);

			program_node_list.push_back(tmp_s);
		}
	}

	return 1;
}

int Interface::clear_all_configuration_lists()
{
	// clearing of lists
	section_list.clear();
	config_node_list.clear();
	all_node_list.clear();
	program_node_list.clear();

	return 1;
}

int Interface::initiate_configuration()
{

	std::cerr << "ui 1" << std::endl;

	if (access(config_file_relativepath.c_str(), R_OK) != 0) {
		fprintf(stderr, "Wrong entry in default_file.cfg - load another configuration than: %s\n", config_file_relativepath.c_str());
		config_file_relativepath = mrrocpp_bin_to_root_path + "configs/common.ini";
	}

	// sprawdzenie czy nazwa sesji jest unikalna

	bool wyjscie = false;

	while (!wyjscie) {
		time_t now = time(NULL);
		char now_string[32];
		strftime(now_string, 8, "_%H%M%S", localtime(&now));
		session_name = now_string;

		if (config) {
			delete config;
		}
		config = new lib::configurator(ui_node_name, mrrocpp_local_path, config_file, lib::UI_SECTION, session_name);

		std::string attach_point =
				config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", lib::UI_SECTION);

		// wykrycie identycznych nazw sesji
		wyjscie = true;

		DIR* dirp = opendir("/dev/name/global");

		if (dirp != NULL) {
			for (;;) {
				struct dirent* direntp = readdir(dirp);
				if (direntp == NULL)
					break;

				// printf( "%s\n", direntp->d_name );
				if (attach_point == direntp->d_name) {
					wyjscie = false;
				}
			}

			closedir(dirp);

		}

	}

	ui_attach_point
			= config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "ui_attach_point", lib::UI_SECTION);
	sr_attach_point
			= config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", lib::UI_SECTION);
	network_sr_attach_point
			= config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", lib::UI_SECTION);

	clear_all_configuration_lists();

	// sczytanie listy sekcji
	fill_section_list(config_file_relativepath.c_str());
	fill_section_list((mrrocpp_bin_to_root_path + "configs/common.ini").c_str());
	fill_node_list();
	fill_program_node_list();

	return 1;
}

// fills section list of configuration files
int Interface::fill_section_list(const char* file_name_and_path)
{
	static char line[256];

	// otworz plik konfiguracyjny
	FILE * file = fopen(file_name_and_path, "r");
	if (file == NULL) {
		printf("UI fill_section_list Wrong file_name: %s\n", file_name_and_path);
		/* TR
		 PtExit(EXIT_SUCCESS);
		 */
	}

	// sczytaj nazwy wszytkich sekcji na liste dynamiczna
	char * fptr = fgets(line, 255, file); // get input line

	// dopoki nie osiagnieto konca pliku

	while (!feof(file)) {
		// jesli znaleziono nowa sekcje
		if ((fptr != NULL) && (line[0] == '[')) {
			char current_section[50];
			strncpy(current_section, line, strlen(line) - 1);
			current_section[strlen(line) - 1] = '\0';

			std::list <Interface::list_t>::iterator list_iterator;

			// checking if section is already considered
			for (list_iterator = section_list.begin(); list_iterator != section_list.end(); list_iterator++) {
				if ((*list_iterator) == current_section)
					break;
			}

			// if the section does not exists
			if (list_iterator == section_list.end()) {
				section_list.push_back(std::string(current_section));
			}

		} // end 	if (( fptr!=NULL )&&( line[0]=='[' ))

		// odczytaj nowa lnie
		fptr = fgets(line, 255, file); // get input line
	} // end while (!feof(file)	)

	// zamknij plik
	fclose(file);

	return 1;
}

// fills node list
void Interface::fill_node_list()
{
	// fill all network nodes list

	DIR* dirp = opendir("/net");
	if (dirp != NULL) {
		for (;;) {
			struct dirent *direntp = readdir(dirp);
			if (direntp == NULL)
				break;
			all_node_list.push_back(std::string(direntp->d_name));
		}
		closedir(dirp);
	}

	for (std::list <Interface::list_t>::iterator section_list_iterator = section_list.begin(); section_list_iterator
			!= section_list.end(); section_list_iterator++) {
		if (config->exists("node_name", *section_list_iterator)) {
			std::string tmp = config->value <std::string> ("node_name", *section_list_iterator);

			std::list <Interface::list_t>::iterator node_list_iterator;

			for (node_list_iterator = config_node_list.begin(); node_list_iterator != config_node_list.end(); node_list_iterator++) {
				if (tmp == (*node_list_iterator)) {
					break;
				}
			}

			// if the node does not exists
			if (node_list_iterator == config_node_list.end()) {
				config_node_list.push_back(tmp);
			}
		}

	}
}

int Interface::execute_mp_pulse(char pulse_code)
{

	// printf("w send pulse\n");
	if (mp.pulse_fd > 0) {
		long pulse_value = 1;

#if !defined(USE_MESSIP_SRR)
		if (MsgSendPulse(mp.pulse_fd, sched_get_priority_min(SCHED_FIFO), pulse_code, pulse_value) == -1)
#else
		if(messip::port_send_pulse(mp.pulse_fd, pulse_code, pulse_value))
#endif
		{
			perror("Blad w wysylaniu pulsu do mp");
			fprintf(stderr, "Blad w wysylaniu pulsu do mp error: %s \n", strerror(errno));
			delay(1000);
		}
	}

	return 1;
}

void Interface::create_threads()
{
	meb_tid = new feb_thread(*main_eb);
	/* TR
	 ui_ecp_obj = new ecp_buffer(*this);
	 */
	delay(1);
	ui_sr_obj = new sr_buffer(*this);

#if defined(__QNXNTO__)

#endif

}

int Interface::EDP_all_robots_create()

{

	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->edp_create();
				}

	return 1;

}

int Interface::EDP_all_robots_slay()

{

	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->EDP_slay_int();
				}

	return 1;

}

int Interface::EDP_all_robots_synchronise()

{

	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->synchronise();
				}

	return 1;

}

int Interface::MPup()

{
	//eb.command(boost::bind(&ui::spkm::UiRobot::execute_motor_motion, &(*this)));
	main_eb->command(boost::bind(&ui::common::Interface::MPup_int, &(*this)));

	return 1;

}

int Interface::MPslay()

{

	if (mp.pid != -1) {

		if ((mp.state == ui::common::UI_MP_TASK_RUNNING) || (mp.state == ui::common::UI_MP_TASK_PAUSED)) {

			pulse_stop_mp();
		}

		messip::port_disconnect(mp.pulse_fd);

		// 	printf("dddd: %d\n", SignalKill(ini_con->mp-
		// 	printf("mp slay\n");
		/* TR
		 SignalKill(mp.node_nr, mp.pid, 0, SIGTERM, 0, 0);
		 */
		mp.state = ui::common::UI_MP_PERMITED_TO_RUN; // mp wylaczone

	}
	// delay(1000);
	// 	kill(mp_pid,SIGTERM);
	// 	printf("mp pupa po kill\n");
	mp.pid = -1;
	mp.pulse_fd = lib::invalid_fd;

	/* TR
	 irp6ot_m->deactivate_ecp_trigger();
	 irp6p_m->deactivate_ecp_trigger();
	 conveyor->deactivate_ecp_trigger();
	 speaker->deactivate_ecp_trigger();
	 irp6m_m->deactivate_ecp_trigger();
	 */
	// modyfikacja menu
	manage_interface();
	/* TR
	 process_control_window_init(widget, apinfo, cbinfo);
	 */
	return 1;

}

int Interface::pulse_start_mp()

{

	if (mp.state == ui::common::UI_MP_WAITING_FOR_START_PULSE) {

		mp.state = ui::common::UI_MP_TASK_RUNNING;// czekanie na stop

		// close_all_windows

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
					{
						robot_node.second->close_all_windows();
					}

		execute_mp_pulse(MP_START);

		/* TR
		 process_control_window_init(widget, apinfo, cbinfo);
		 */
		manage_interface();
	}

	return 1;

}

int Interface::pulse_stop_mp()

{

	if ((mp.state == ui::common::UI_MP_TASK_RUNNING) || (mp.state == ui::common::UI_MP_TASK_PAUSED)) {

		mp.state = ui::common::UI_MP_WAITING_FOR_START_PULSE;// czekanie na stop

		execute_mp_pulse(MP_STOP);
		/* TR
		 process_control_window_init(widget, apinfo, cbinfo);
		 */
		manage_interface();
	}

	return 1;

}

int Interface::pulse_pause_mp()

{

	if (mp.state == ui::common::UI_MP_TASK_RUNNING) {

		mp.state = ui::common::UI_MP_TASK_PAUSED;// czekanie na stop

		execute_mp_pulse(MP_PAUSE);
		/* TR
		 process_control_window_init(widget, apinfo, cbinfo);
		 */
		manage_interface();
	}

	return 1;

}

int Interface::pulse_resume_mp()

{

	if (mp.state == ui::common::UI_MP_TASK_PAUSED) {

		mp.state = ui::common::UI_MP_TASK_RUNNING;// czekanie na stop

		execute_mp_pulse(MP_RESUME);
		/* TR
		 process_control_window_init(widget, apinfo, cbinfo);
		 */
		manage_interface();
	}

	return 1;

}

int Interface::pulse_trigger_mp()

{

	if (mp.state == ui::common::UI_MP_TASK_RUNNING) {

		execute_mp_pulse(MP_TRIGGER);
		/* TR
		 process_control_window_init(widget, apinfo, cbinfo);
		 */
		manage_interface();
	}

	return 1;

}

// zatrzymuje zadanie, zabija procesy
int Interface::unload_all()

{

	MPslay();
	delay(200);
	EDP_all_robots_slay();
	/* TR
	 close_process_control_window(widget, apinfo, cbinfo);
	 */
	return 1;

}

// najpierw unload_all zabija wszystkie procesy wzmiankowane w pliku konfiguracyjnym

int Interface::slay_all()
{

	// program unload

	unload_all();

	// brutal overkilling

	for (std::list <ui::common::program_node_def>::iterator program_node_list_iterator = program_node_list.begin(); program_node_list_iterator
			!= program_node_list.end(); program_node_list_iterator++) {
		char system_command[100];
		/*
		 #if 0
		 sprintf(system_command, "rsh %s killall -e -q -v %s",
		 program_node_list_iterator->node_name,
		 program_node_list_iterator->program_name
		 );
		 #else
		 sprintf(system_command, "slay -v -f -n %s %s",
		 program_node_list_iterator->node_name,
		 program_node_list_iterator->program_name
		 );
		 #endif
		 printf("aaa: %s\n", system_command);
		 system(system_command);
		 */
		delay(10);

#if 0
		sprintf(system_command, "rsh %s killall -e -q -v %s",
				program_node_list_iterator->node_name.c_str(),
				program_node_list_iterator->program_name.c_str()
		);
#else
		sprintf(system_command, "killall -e -q -v %s", program_node_list_iterator->program_name.c_str());
#endif
		printf("bbb: %s\n", system_command);
		system(system_command);
	}

	manage_interface();

	return 1;

}

int Interface::all_robots_move_to_synchro_position()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((mp.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN) || (mp.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (mp.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
					{
						if (robot_node.second->check_synchronised_and_loaded()) {
							robot_node.second->move_to_synchro_position();
						}
					}

	}

	return 1;

}

int Interface::all_robots_move_to_preset_position_1()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((mp.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN) || (mp.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (mp.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
					{
						if (robot_node.second->check_synchronised_and_loaded()) {
							robot_node.second->move_to_preset_position(1);
						}
					}

	}

	return 1;

}

int Interface::all_robots_move_to_preset_position_2()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((mp.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN) || (mp.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (mp.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
					{
						if (robot_node.second->check_synchronised_and_loaded()) {
							robot_node.second->move_to_preset_position(2);
						}
					}

	}
	return 1;

}

int Interface::all_robots_move_to_preset_position_0()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((mp.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN) || (mp.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (mp.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
					{
						if (robot_node.second->check_synchronised_and_loaded()) {
							robot_node.second->move_to_preset_position(0);
						}
					}

	}

	return 1;

}

int Interface::all_robots_move_to_front_position()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((mp.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN) || (mp.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (mp.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
					{
						if (robot_node.second->check_synchronised_and_loaded()) {
							robot_node.second->move_to_front_position();
						}
					}

	}
	return 1;

}

}
}
}
