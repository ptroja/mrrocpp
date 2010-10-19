/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                     Version 2.01  */
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <strings.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <csignal>
#include <sys/netmgr.h>
#include <cerrno>
#include <spawn.h>
#include <process.h>
#include <cassert>
#include <iostream>
#include <fstream>

// niezbedny naglowek z definiacja PROCESS_SPAWN_RSH
#include "base/lib/configurator.h"
#include "base/lib/mis_fun.h"

#include "ui/src/ui_class.h"
#include "ui/src/ui_ecp.h"
#include "ui/src/ui_sr.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

//
//
// KLASA ui
//
//

#if defined(USE_MESSIP_SRR)
#include "base/lib/messip/messip_dataport.h"
#endif

namespace mrrocpp {
namespace ui {
namespace common {

Interface::Interface() :
	config(NULL), all_ecp_msg(NULL), ui_msg(NULL), is_mp_and_ecps_active(false), all_edps(UI_ALL_EDPS_NONE_EDP_LOADED)
{

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

}

void Interface::init()
{

	// ustalenie katalogow UI

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

	bird_hand = new bird_hand::UiRobot(*this);
	irp6ot_m = new irp6ot_m::UiRobot(*this);
	irp6ot_tfg = new irp6ot_tfg::UiRobot(*this);
	irp6p_m = new irp6p_m::UiRobot(*this);
	irp6p_tfg = new irp6p_tfg::UiRobot(*this);
	sarkofag = new sarkofag::UiRobot(*this);
	irp6m_m = new irp6m::UiRobot(*this);
	conveyor = new conveyor::UiRobot(*this);
	speaker = new speaker::UiRobot(*this);
	spkm = new spkm::UiRobot(*this);
	smb = new smb::UiRobot(*this);
	shead = new shead::UiRobot(*this);

	ui_node_name = sysinfo.nodename;
	is_sr_thread_loaded = false;

	binaries_local_path = cwd;
	mrrocpp_local_path = cwd;
	mrrocpp_local_path.erase(mrrocpp_local_path.length() - 3);// kopiowanie lokalnej sciezki bez "bin" - 3 znaki
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
	//	config_file_fullpath += "configs";

	// printf ("Remember to create gns server\n");


	set_ui_state_notification(UI_N_STARTING);

	signal(SIGINT, &catch_signal);// by y aby uniemozliwic niekontrolowane zakonczenie aplikacji ctrl-c z kalwiatury
	signal(SIGALRM, &catch_signal);
	signal(SIGSEGV, &catch_signal);
#ifdef PROCESS_SPAWN_RSH
	signal(SIGCHLD, &catch_signal);
#endif /* PROCESS_SPAWN_RSH */

	lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 6);

	// pierwsze zczytanie pliku konfiguracyjnego (aby pobrac nazwy dla pozostalych watkow UI)
	if (get_default_configuration_file_name() >= 1) // zczytaj nazwe pliku konfiguracyjnego
	{
		initiate_configuration();
		// sprawdza czy sa postawione gns's i ew. stawia je
		// uwaga serwer musi byc wczesniej postawiony
		check_gns();
	} else {
		printf("Blad manage_default_configuration_file\n");
		PtExit(EXIT_SUCCESS);
	}

	create_threads();

	// Zablokowanie domyslnej obslugi sygnalu SIGINT w watkach UI_SR i UI_COMM


	// kolejne zczytanie pliku konfiguracyjnego
	if (get_default_configuration_file_name() == 1) // zczytaj nazwe pliku konfiguracyjnego
	{
		reload_whole_configuration();

	} else {
		printf("Blad manage_default_configuration_file\n");
		PtExit(EXIT_SUCCESS);
	}

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
	strcpy(log_file_with_dir, "../logs/");

	if (access(log_file_with_dir, R_OK) != 0) {
		mkdir(log_file_with_dir, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
	}

	strcat(log_file_with_dir, file_name);

	log_file_outfile = new std::ofstream(log_file_with_dir, std::ios::out);

	if (!(*log_file_outfile)) {
		std::cerr << "Cannot open file: " << file_name << '\n';
		perror("because of");
	}

	manage_interface();

}

// funkcja odpowiedzialna za wyglad aplikacji na podstawie jej stanu

int Interface::manage_interface(void)
{
	int pt_res;
	pt_res = PtEnter(0);

	check_edps_state_and_modify_mp_state();

	// na wstepie wylaczamy przyciski EDP z all robots menu. Sa one ewentualnie wlaczane dalej
	ApModifyItemState(&all_robots_menu, AB_ITEM_DIM, ABN_mm_all_robots_preset_positions, ABN_mm_all_robots_synchronisation, ABN_mm_all_robots_edp_unload, ABN_mm_all_robots_edp_load, NULL);

	// menu file
	// ApModifyItemState( &file_menu, AB_ITEM_DIM, NULL);

	// Dla robota IRP6 ON_TRACK
	irp6ot_m->manage_interface();
	irp6ot_tfg->manage_interface();

	// Dla robota IRP6 POSTUMENT
	irp6p_m->manage_interface();
	irp6p_tfg->manage_interface();
	sarkofag->manage_interface();

	// Dla robota CONVEYOR
	conveyor->manage_interface();

	// ROBOTY SwamrmItFix
	spkm->manage_interface();
	smb->manage_interface();
	shead->manage_interface();

	bird_hand->manage_interface();

	// Dla robota SPEAKER
	speaker->manage_interface();

	// Dla robota IRP6 MECHATRONIKA
	irp6m_m->manage_interface();

	// zadanie
	// kolorowanie menu all robots


	// wlasciwosci menu  ABW_base_all_robots


	switch (all_edps)
	{
		case UI_ALL_EDPS_NONE_EDP_ACTIVATED:
			//				printf("UI_ALL_EDPS_NONE_EDP_ACTIVATED\n");
			block_widget(ABW_base_all_robots);
			PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_GRAY, 0);
			block_widget(ABW_base_robot);
			PtSetResource(ABW_base_robot, Pt_ARG_COLOR, Pg_GRAY, 0);
			break;
		case UI_ALL_EDPS_NONE_EDP_LOADED:
			//				printf("UI_ALL_EDPS_NONE_EDP_LOADED\n");
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_load, NULL);
			PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_BLACK, 0);
			PtSetResource(ABW_base_robot, Pt_ARG_COLOR, Pg_BLACK, 0);
			unblock_widget(ABW_base_all_robots);
			unblock_widget(ABW_base_robot);
			break;
		case UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED:
			//			printf("UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED\n");
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_unload, NULL);
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_load, NULL);
			PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_BLACK, 0);
			PtSetResource(ABW_base_robot, Pt_ARG_COLOR, Pg_BLACK, 0);
			unblock_widget(ABW_base_all_robots);
			unblock_widget(ABW_base_robot);
			break;
		case UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED:
			//			printf("UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED\n");
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_unload, NULL);
			PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_DBLUE, 0);
			unblock_widget(ABW_base_all_robots);
			unblock_widget(ABW_base_robot);
			break;
		case UI_ALL_EDPS_LOADED_AND_SYNCHRONISED:
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
			break;
		default:
			break;
	}

	// wlasciwosci menu task_menu
	switch (mp.state)
	{

		case common::UI_MP_NOT_PERMITED_TO_RUN:
			ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_load, ABN_mm_mp_unload, NULL);
			PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_BLACK, 0);
			break;
		case common::UI_MP_PERMITED_TO_RUN:
			ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_unload, NULL);
			ApModifyItemState(&task_menu, AB_ITEM_NORMAL, ABN_mm_mp_load, NULL);
			PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_BLACK, 0);
			break;
		case common::UI_MP_WAITING_FOR_START_PULSE:
			ApModifyItemState(&task_menu, AB_ITEM_NORMAL, ABN_mm_mp_unload, NULL);
			ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_load, NULL);
			//	ApModifyItemState( &all_robots_menu, AB_ITEM_DIM, ABN_mm_all_robots_edp_unload, NULL);
			PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_DBLUE, 0);
			break;
		case common::UI_MP_TASK_RUNNING:
		case common::UI_MP_TASK_PAUSED:
			ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_unload, ABN_mm_mp_load, NULL);
			PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_BLUE, 0);
			break;
		default:
			break;
	}

	//	PtFlush();

	if (pt_res >= 0)
		PtLeave(0);

	return 1;
}

void Interface::reload_whole_configuration()
{

	if (access(config_file_relativepath.c_str(), R_OK) != 0) {
		std::cerr << "Wrong entry in default_file.cfg - load another configuration than: " << config_file_relativepath << std::endl;
		config_file_relativepath = "../configs/common.ini";
	}

	if ((mp.state == UI_MP_NOT_PERMITED_TO_RUN) || (mp.state == UI_MP_PERMITED_TO_RUN)) { // jesli nie dziala mp podmien mp ecp vsp

		config->change_config_file(config_file);

		is_mp_and_ecps_active = config->value <int> ("is_mp_and_ecps_active");

		switch (all_edps)
		{
			case UI_ALL_EDPS_NONE_EDP_ACTIVATED:
			case UI_ALL_EDPS_NONE_EDP_LOADED:

				// dla robota irp6 on_track
				irp6ot_m->reload_configuration();
				irp6ot_tfg->reload_configuration();

				// dla robota irp6 postument
				irp6p_m->reload_configuration();
				irp6p_tfg->reload_configuration();

				sarkofag->reload_configuration();

				// dla robota conveyor
				conveyor->reload_configuration();

				// ROBOTY SwamrmItFix
				spkm->reload_configuration();
				smb->reload_configuration();
				shead->reload_configuration();

				bird_hand->reload_configuration();

				// dla robota speaker
				speaker->reload_configuration();

				// dla robota irp6 mechatronika
				irp6m_m->reload_configuration();
				break;
			default:
				break;
		}

		// clearing of lists
		clear_all_configuration_lists();

		// sczytanie listy sekcji
		fill_section_list(config_file_relativepath.c_str());
		fill_section_list("../configs/common.ini");
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
	std::string opendir_path("/net/");
	opendir_path += _node;

	if (access(opendir_path.c_str(), R_OK) != 0) {
		std::string tmp(beginnig_of_message);
		tmp += std::string(" node: ") + _node + std::string(" is unreachable");
		ui_msg->message(lib::NON_FATAL_ERROR, tmp);

		return false;
	}
	return true;
}

// sprawdza czy sa postawione gns's i ew. stawia je
// uwaga serwer powinien byc wczesniej postawiony (dokladnie jeden w sieci)

int Interface::check_gns()
{
	if (access("/etc/system/config/useqnet", R_OK)) {
		printf("UI: There is no /etc/system/config/useqnet file; the qnet will not work properly.\n");
		PtExit(EXIT_SUCCESS);
	}

	unsigned short number_of_gns_servers = 0;
	std::string gns_server_node;

	// poszukiwanie serwerow gns
	for (std::list <Interface::list_t>::iterator node_list_iterator = all_node_list.begin(); node_list_iterator
			!= all_node_list.end(); node_list_iterator++) {
		std::string opendir_path("/net/");

		opendir_path += *node_list_iterator;
		opendir_path += "/proc/mount/dev/name/gns_server";

		// sprawdzenie czy dziala serwer gns

		if (access(opendir_path.c_str(), R_OK) == 0) {
			number_of_gns_servers++;
			gns_server_node = *node_list_iterator;
		}
	}

	// there is more than one gns server in the QNX network
	if (number_of_gns_servers > 1) {
		printf("UI: There is more than one gns server in the QNX network; the qnet will not work properly.\n");
		// printing of gns server nodes
		for (std::list <Interface::list_t>::iterator node_list_iterator = all_node_list.begin(); node_list_iterator
				!= all_node_list.end(); node_list_iterator++) {
			std::string opendir_path("/net/");

			opendir_path += *node_list_iterator;
			opendir_path += "/proc/mount/dev/name/gns_server";

			// sprawdzenie czy dziala serwer gns
			if (access(opendir_path.c_str(), R_OK) == 0) {
				printf("There is gns server on %s node\n", (*node_list_iterator).c_str());
			}
		}
		PtExit(EXIT_SUCCESS);
	}
	// gns server was not found in the QNX network
	else if (!number_of_gns_servers) {
		printf("UI: gns server was not found in the QNX network, it will be automatically run on local node\n");

		// ew. zabicie klienta gns

		if (access("/dev/name", R_OK) == 0) {
			system("slay gns");
		}

		// uruchomienie serwera
		system("gns -s");

		// poszukiwanie serwerow gns
		for (std::list <Interface::list_t>::iterator node_list_iterator = all_node_list.begin(); node_list_iterator
				!= all_node_list.end(); ++node_list_iterator) {
			std::string opendir_path("/net/");

			opendir_path += *node_list_iterator;
			opendir_path += "/proc/mount/dev/name/gns_server";
			//	strcat(opendir_path, "/dev/name/gns_server");

			// sprawdzenie czy dziala serwer gns
			if (access(opendir_path.c_str(), R_OK) == 0) {
				number_of_gns_servers++;
				gns_server_node = *node_list_iterator;
			}
		}
	}

	// sprawdzanie lokalne

	if (access("/proc/mount/dev/name", R_OK) != 0) {
		std::string system_command("gns -c ");
		system_command += gns_server_node;
		system(system_command.c_str());
	}

	// sprawdzenie czy wezly w konfiuracji sa uruchomione i ew. uruchomienie na nich brakujacych klientow gns
	for (std::list <Interface::list_t>::iterator node_list_iterator = config_node_list.begin(); node_list_iterator
			!= config_node_list.end(); node_list_iterator++) {
		std::string opendir_path("/net/");

		opendir_path += *node_list_iterator;

		// sprawdzenie czy istnieje wezel
		if (access(opendir_path.c_str(), R_OK) == 0) {
			opendir_path += "/proc/mount/dev/name";

			// sprawdzenie czy dziala gns
			if (access(opendir_path.c_str(), R_OK) != 0) {
				std::string system_command("on -f ");

				system_command += *node_list_iterator;
				system_command += " gns -c ";
				system_command += gns_server_node;

				std::cerr << "SYSTEM(" << system_command << ")" << std::endl;
				system(system_command.c_str());
			}

		} else {
			fprintf(stderr, "check_gns - Nie wykryto wezla: %s, ktory wystepuje w pliku konfiguracyjnym\n", (*node_list_iterator).c_str());

			if ((is_sr_thread_loaded) && (ui_msg != NULL)) {
				std::string tmp;
				tmp = std::string("check_gns - Nie wykryto wezla: ") + (*node_list_iterator)
						+ std::string(", ktory wystepuje w pliku konfiguracyjnym");
				ui_msg->message(lib::NON_FATAL_ERROR, tmp);
			}

		}
	}

	return (Pt_CONTINUE);
}

// ustala stan wszytkich EDP
int Interface::check_edps_state_and_modify_mp_state()
{

	// wyznaczenie stanu wszytkich EDP abstahujac od MP

	// jesli wszytkie sa nieaktywne
	if ((!(irp6p_m->state.is_active)) && (!(irp6ot_m->state.is_active)) && (!(irp6ot_tfg->state.is_active))
			&& (!(irp6p_tfg->state.is_active)) && (!(sarkofag->state.is_active)) && (!(conveyor->state.is_active))
			&& (!(speaker->state.is_active)) && (!(irp6m_m->state.is_active)) && (!(bird_hand->state.is_active))
			&& (!(spkm->state.is_active)) && (!(smb->state.is_active)) && (!(shead->state.is_active))) {
		all_edps = UI_ALL_EDPS_NONE_EDP_ACTIVATED;

		// jesli wszystkie sa zsynchronizowane
	} else if (check_synchronised_or_inactive(irp6p_m->state) && check_synchronised_or_inactive(irp6ot_m->state)
			&& check_synchronised_or_inactive(conveyor->state) && check_synchronised_or_inactive(speaker->state)
			&& check_synchronised_or_inactive(irp6m_m->state) && check_synchronised_or_inactive(irp6ot_tfg->state)
			&& check_synchronised_or_inactive(irp6p_tfg->state) && check_synchronised_or_inactive(sarkofag->state)
			&& check_synchronised_or_inactive(bird_hand->state) && check_synchronised_or_inactive(spkm->state)
			&& check_synchronised_or_inactive(smb->state) && check_synchronised_or_inactive(shead->state)) {
		all_edps = UI_ALL_EDPS_LOADED_AND_SYNCHRONISED;

		// jesli wszystkie sa zaladowane
	} else if (check_loaded_or_inactive(irp6p_m->state) && check_loaded_or_inactive(irp6ot_m->state)
			&& check_loaded_or_inactive(conveyor->state) && check_loaded_or_inactive(speaker->state)
			&& check_loaded_or_inactive(irp6m_m->state) && check_loaded_or_inactive(irp6ot_tfg->state)
			&& check_loaded_or_inactive(irp6p_tfg->state) && check_loaded_or_inactive(sarkofag->state)
			&& check_loaded_or_inactive(bird_hand->state) && check_loaded_or_inactive(spkm->state)
			&& check_loaded_or_inactive(smb->state) && check_loaded_or_inactive(shead->state))

	{
		all_edps = UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED;

		// jesli chociaz jeden jest zaladowany
	} else if (check_loaded(irp6p_m->state) || check_loaded(irp6ot_m->state) || check_loaded(conveyor->state)
			|| check_loaded(speaker->state) || check_loaded(irp6m_m->state) || check_loaded(irp6ot_tfg->state)
			|| check_loaded(irp6p_tfg->state) || check_loaded(sarkofag->state) || check_loaded(bird_hand->state)
			|| check_loaded(spkm->state) || check_loaded(smb->state) || check_loaded(shead->state))

	{
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

// ustala stan wszytkich EDP
bool Interface::check_synchronised_or_inactive(ecp_edp_ui_robot_def& robot)
{
	return (((robot.is_active) && (robot.edp.is_synchronised)) || (!(robot.is_active)));

}

bool Interface::check_synchronised_and_loaded(ecp_edp_ui_robot_def& robot)
{
	return (((robot.edp.state > 0) && (robot.edp.is_synchronised)));

}

bool Interface::check_loaded_or_inactive(ecp_edp_ui_robot_def& robot)
{
	return (((robot.is_active) && (robot.edp.state > 0)) || (!(robot.is_active)));

}

bool Interface::check_loaded(ecp_edp_ui_robot_def& robot)
{
	return ((robot.is_active) && (robot.edp.state > 0));
}

// odczytuje nazwe domyslengo pliku konfiguracyjnego, w razie braku ustawia common.ini
int Interface::get_default_configuration_file_name()
{

	FILE * fp = fopen("../configs/default_file.cfg", "r");
	if (fp != NULL) {
		//printf("alala\n");
		char tmp_buf[255];
		fgets(tmp_buf, 255, fp); // Uwaga na zwracanego NULLa
		char *tmp_buf1 = strtok(tmp_buf, "=\n\r"); // get first token
		config_file = tmp_buf1;

		config_file_relativepath = "../";
		config_file_relativepath += config_file;

		fclose(fp);
		return 1;

	} else {
		//	printf("balala\n");
		// jesli plik z domyslna konfiguracja (default_file.cfg) nie istnieje to utworz go i wpisz do niego common.ini
		printf("Utworzono plik default_file.cfg z konfiguracja common.ini\n");
		fp = fopen("../configs/default_file.cfg", "w");
		fclose(fp);

		config_file = "configs/common.ini";
		config_file_relativepath = "../";
		config_file_relativepath += config_file;

		std::ofstream outfile("../configs/default_file.cfg", std::ios::out);
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

	config_file_relativepath = "../";
	config_file_relativepath += config_file;

	std::ofstream outfile("../configs/default_file.cfg", std::ios::out);
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
	if (access(config_file_relativepath.c_str(), R_OK) != 0) {
		fprintf(stderr, "Wrong entry in default_file.cfg - load another configuration than: %s\n", config_file_relativepath.c_str());
		config_file_relativepath = "../configs/common.ini";
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
	fill_section_list("../configs/common.ini");
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
		PtExit(EXIT_SUCCESS);
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

void Interface::pulse_reader_execute(edp_state_def::reader_fd_t coid, int code, int value)
{
#if !defined(USE_MESSIP_SRR)
	if(MsgSendPulse(coid, sched_get_priority_min(SCHED_FIFO), code, value) == -1)
#else
	if(messip::port_send_pulse(coid, code, value))
#endif
	{
		perror("Blad w wysylaniu pulsu do redera");
	}
}

int Interface::execute_mp_pulse(char pulse_code)
{
	int ret = -2;

	// printf("w send pulse\n");
	if (mp.pulse_fd > 0) {
		long pulse_value = 1;
		if ((ret = MsgSendPulse(mp.pulse_fd, sched_get_priority_min(SCHED_FIFO), pulse_code, pulse_value)) == -1) {

			perror("Blad w wysylaniu pulsu do mp");
			fprintf(stderr, "Blad w wysylaniu pulsu do mp error: %s \n", strerror(errno));
			delay(1000);
		}
	}

	return ret;
}

bool Interface::deactivate_ecp_trigger(ecp_edp_ui_robot_def& robot_l)
{

	if (robot_l.is_active) {
		if (robot_l.ecp.trigger_fd >= 0) {
			name_close(robot_l.ecp.trigger_fd);
		}
		robot_l.ecp.trigger_fd = -1;
		robot_l.ecp.pid = -1;
		return true;
	}

	return false;
}

int Interface::set_toggle_button(PtWidget_t * widget)
{

	PtSetResource(widget, Pt_ARG_FLAGS, Pt_TRUE, Pt_SET);
	PtDamageWidget(widget);

	return 1;
}

int Interface::unset_toggle_button(PtWidget_t * widget)
{

	PtSetResource(widget, Pt_ARG_FLAGS, Pt_FALSE, Pt_SET);
	PtDamageWidget(widget);

	return 1;
}

// blokowanie widgetu
int Interface::block_widget(PtWidget_t *widget)
{
	PtSetResource(widget, Pt_ARG_FLAGS, Pt_TRUE, Pt_BLOCKED | Pt_GHOST);
	PtDamageWidget(widget);

	return 1;
}

// odblokowanie widgetu
int Interface::unblock_widget(PtWidget_t *widget)
{
	PtSetResource(widget, Pt_ARG_FLAGS, Pt_FALSE, Pt_BLOCKED | Pt_GHOST);
	PtDamageWidget(widget);

	return 1;
}

void Interface::create_threads()

{
	meb_tid = new feb_thread(main_eb);
	ui_ecp_obj = new ecp_buffer(*this);
	delay(1);
	ui_sr_obj = new sr_buffer(*this);

#if defined(__QNXNTO__)

#endif

}

}
} //namespace ui
} //namespace mrrocpp
