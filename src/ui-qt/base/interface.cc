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
#include "ui_mainwindow.h"

#include "interface.h"
#include "ui_sr.h"
#include "ui_ecp.h"
#include "../spkm/ui_r_spkm.h"
#include "../smb/ui_r_smb.h"
#include "../shead/ui_r_shead.h"
#include "../irp6ot_m/ui_r_irp6ot_m.h"
#include "../irp6p_m/ui_r_irp6p_m.h"
#include "../irp6p_tfg/ui_r_irp6p_tfg.h"
#include "../irp6ot_tfg/ui_r_irp6ot_tfg.h"
#include "../polycrank/ui_r_polycrank.h"
#include "../bird_hand/ui_r_bird_hand.h"
#include "../sarkofag/ui_r_sarkofag.h"
#include "../conveyor/ui_r_conveyor.h"

extern void catch_signal(int sig);

namespace mrrocpp {
namespace ui {
namespace common {

Interface::Interface() :
	config(NULL), is_mp_and_ecps_active(false), all_edps(UI_ALL_EDPS_NONE_LOADED),
			all_edps_last_manage_interface_state(UI_ALL_EDPS_STATE_NOT_KNOWN),
			all_edps_synchro(UI_ALL_EDPS_NONE_SYNCHRONISED),
			all_edps_synchro_last_manage_interface_state(UI_ALL_EDPS_SYNCHRO_STATE_NOT_KNOWN),
			position_refresh_interval(200)
{

	mw = new MainWindow(*this);

	main_eb = new function_execution_buffer(*this);

	connect(this, SIGNAL(manage_interface_signal()), this, SLOT(manage_interface_slot()), Qt::QueuedConnection);

	mp.state = UI_MP_NOT_PERMITED_TO_RUN;// mp wylaczone
	mp.last_process_control_state = UI_MP_STATE_NOT_KNOWN;
	mp.last_manage_interface_state = UI_MP_STATE_NOT_KNOWN;

	mp.pid = -1;

	ui_state = 1;// ui working
	file_window_mode = ui::common::FSTRAJECTORY; // uczenie


	mrrocpp_bin_to_root_path = "../../";

}

//Interface * Interface::get_instance()
//{
//	static Interface *instance = new Interface();
//	return instance;
//}


MainWindow* Interface::get_main_window()
{
	return mw;
}

int Interface::set_ui_state_notification(UI_NOTIFICATION_STATE_ENUM new_notifacion)
{

	{

		boost::unique_lock <boost::mutex> lock(ui_notification_state_mutex);

		next_notification = new_notifacion;

	}

	mw->ui_notification();

	return 1;

}

void Interface::init()
{

	// dodanie innych okien w dock widgetach
	wgt_pc = new wgt_process_control(*this);
	wgt_yes_no_obj = new wgt_yes_no(*this);
	wgt_message_obj = new wgt_message(*this);
	wgt_input_integer_obj = new wgt_input_integer(*this);
	wgt_input_double_obj = new wgt_input_double(*this);
	wgt_choose_option_obj = new wgt_choose_option(*this);
	wgt_teaching_obj = new wgt_teaching(*this);

	// ustalenie katalogow UI

	mw->show();

	/*
	 dwgt_pc = new QDockWidget(mw);
	 //dwgt_pc->setAllowedAreas(Qt::TopDockWidgetArea);
	 dwgt_pc->setWindowTitle("Process control");

	 vl_pc = new QVBoxLayout();
	 dwgt_pc->setLayout(vl_pc);

	 vl_pc->addWidget(wgt_pc);
	 dwgt_pc->setWidget(wgt_pc);
	 dwgt_pc->hide();
	 mw->addDockWidget(Qt::LeftDockWidgetArea, dwgt_pc);
	 */
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

	smb = new smb::UiRobot(*this);
	robot_m[smb->robot_name] = smb;

	shead = new shead::UiRobot(*this);
	robot_m[shead->robot_name] = shead;

	irp6ot_m = new irp6ot_m::UiRobot(*this);
	robot_m[irp6ot_m->robot_name] = irp6ot_m;

	irp6p_m = new irp6p_m::UiRobot(*this);
	robot_m[irp6p_m->robot_name] = irp6p_m;

	polycrank = new polycrank::UiRobot(*this);
	robot_m[polycrank->robot_name] = polycrank;

	bird_hand = new bird_hand::UiRobot(*this);
	robot_m[bird_hand->robot_name] = bird_hand;

	sarkofag = new sarkofag::UiRobot(*this);
	robot_m[sarkofag->robot_name] = sarkofag;

	irp6p_tfg = new irp6p_tfg::UiRobot(*this);
	robot_m[irp6p_tfg->robot_name] = irp6p_tfg;

	conveyor = new conveyor::UiRobot(*this);
	robot_m[conveyor->robot_name] = conveyor;

	irp6ot_tfg = new irp6ot_tfg::UiRobot(*this);
	robot_m[irp6ot_tfg->robot_name] = irp6ot_tfg;

	ui_node_name = sysinfo.nodename;
	is_sr_thread_loaded = false;

	binaries_local_path = cwd;
	mrrocpp_local_path = cwd;
	mrrocpp_local_path.erase(mrrocpp_local_path.length() - 3);// kopiowanie lokalnej sciezki bez "bin" - 3 znaki
	//mrrocpp_local_path += "../";
	mrrocpp_root_local_path = mrrocpp_local_path.substr(0, mrrocpp_local_path.rfind("/"));
	mrrocpp_root_local_path = mrrocpp_root_local_path.substr(0, mrrocpp_root_local_path.rfind("/") + 1);
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

	signal(SIGINT, &catch_signal);// by y aby uniemozliwic niekontrolowane zakonczenie aplikacji ctrl-c z kalwiatury
	signal(SIGALRM, &catch_signal);
	signal(SIGSEGV, &catch_signal);

	signal(SIGCHLD, &catch_signal);
	/* TR
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

//void Interface::print_on_sr(const std::string &text)
void Interface::print_on_sr(const char *buff, ...)
{
	char text[256];
	va_list arglist;

	va_start(arglist,buff);
	vsprintf(text, buff, arglist);
	va_end(arglist);

	ui_msg->message(text);
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

				unsigned tmp = 0;
				// kilka sekund  (~1) na otworzenie urzadzenia
				while ((mp.pulse_fd = messip::port_connect(mp.network_pulse_attach_point)) == lib::invalid_fd) {
					if ((tmp++) < lib::CONNECT_RETRY) {
						usleep(lib::CONNECT_DELAY);
					} else {
						fprintf(stderr, "name_open() for %s failed: %s\n", mp.network_pulse_attach_point.c_str(), strerror(errno));
						break;
					}
				}

				teachingstate = ui::common::MP_RUNNING;

				mp.state = ui::common::UI_MP_WAITING_FOR_START_PULSE; // mp wlaczone


				mw->raise_process_control_window();

			} else {
				fprintf(stderr, "mp spawn failed\n");
			}
			manage_interface();

		}
	}

	return 1;
}

void Interface::manage_pc(void)
{
	wgt_pc->process_control_window_init();

}

// funkcja odpowiedzialna za wyglad aplikacji na podstawie jej stanu

int Interface::manage_interface(void)
{
	emit
	manage_interface_signal();

	return 1;
}

void Interface::manage_interface_slot()
{
	// okienko process control
	wgt_pc->process_control_window_init_slot();
	// UWAGA ta funkcja powinna byc odporna na odpalenie z dowolnego watku !!!

	check_edps_state_and_modify_mp_state();
	/*TR
	 // na wstepie wylaczamy przyciski EDP z all robots menu. Sa one ewentualnie wlaczane dalej
	 ApModifyItemState(&all_robots_menu, AB_ITEM_DIM, ABN_mm_all_robots_preset_positions,
	 ABN_mm_all_robots_synchronisation, ABN_mm_all_robots_edp_unload, ABN_mm_all_robots_edp_load, NULL);

	 // menu file
	 // ApModifyItemState( &file_menu, AB_ITEM_DIM, NULL);

	 */
	// uruchmomienie manage interface dla wszystkich robotow
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->manage_interface();
				}

	// wlasciwosci menu  ABW_base_all_robots

	if (all_edps_synchro != all_edps_synchro_last_manage_interface_state) {
		switch (all_edps_synchro)
		{
			case UI_ALL_EDPS_SYNCHRO_STATE_NOT_KNOWN:
				mw->get_ui()->label_all_edps_synchro_notification->setText("NOT_KNOWN");
				break;
			case UI_ALL_EDPS_SYNCHRO_NONE_EDP_LOADED:
				mw->get_ui()->label_all_edps_synchro_notification->setText("NONE EDP LOADED");
				mw->enable_menu_item(false, 1, mw->get_ui()->actionall_Synchronisation);
				break;
			case UI_ALL_EDPS_NONE_SYNCHRONISED:
				mw->get_ui()->label_all_edps_synchro_notification->setText("NONE_SYNCHRONISED");
				break;
			case UI_ALL_EDPS_SOME_SYNCHRONISED:
				mw->get_ui()->label_all_edps_synchro_notification->setText("SOME_SYNCHRONISED");
				break;
			case UI_ALL_EDPS_ALL_SYNCHRONISED:
				mw->get_ui()->label_all_edps_synchro_notification->setText("ALL_SYNCHRONISED");
				mw->enable_menu_item(false, 1, mw->get_ui()->actionall_Synchronisation);

				break;
		}
	}

	if ((all_edps != all_edps_last_manage_interface_state) || (all_edps_synchro
			!= all_edps_synchro_last_manage_interface_state) || (mp.state != mp.last_manage_interface_state)) {

		switch (all_edps)
		{
			case UI_ALL_EDPS_NONE_ACTIVATED:
				mw->get_ui()->label_all_edps_notification->setText("NONE_ACTIVATED");
				mw->enable_menu_item(false, 1, mw->get_ui()->menuall_Preset_Positions);
				mw->enable_menu_item(false, 2, mw->get_ui()->menuRobot, mw->get_ui()->menuAll_Robots);
				mw->enable_menu_item(false, 2, mw->get_ui()->actionall_EDP_Unload, mw->get_ui()->actionall_EDP_Load);

				break;
			case UI_ALL_EDPS_NONE_LOADED:
				mw->get_ui()->label_all_edps_notification->setText("NONE_LOADED");
				//print_on_sr("UI_ALL_EDPS_NONE_EDP_LOADED");
				mw->enable_menu_item(true, 2, mw->get_ui()->menuRobot, mw->get_ui()->menuAll_Robots);
				mw->enable_menu_item(true, 1, mw->get_ui()->actionall_EDP_Load);
				mw->enable_menu_item(false, 1, mw->get_ui()->menuall_Preset_Positions);
				mw->enable_menu_item(false, 1, mw->get_ui()->actionall_EDP_Unload);

				break;
			case UI_ALL_EDPS_SOME_LOADED:
				mw->get_ui()->label_all_edps_notification->setText("SOME_LOADED");
				mw->enable_menu_item(true, 2, mw->get_ui()->actionall_EDP_Unload, mw->get_ui()->actionall_EDP_Load);
				mw->enable_menu_item(true, 2, mw->get_ui()->menuRobot, mw->get_ui()->menuAll_Robots);

				switch (all_edps_synchro)
				{
					case UI_ALL_EDPS_SYNCHRO_STATE_NOT_KNOWN:
					case UI_ALL_EDPS_NONE_SYNCHRONISED:
						mw->enable_menu_item(false, 1, mw->get_ui()->menuall_Preset_Positions);
						mw->enable_menu_item(true, 1, mw->get_ui()->actionall_Synchronisation);

						break;
					case UI_ALL_EDPS_SOME_SYNCHRONISED:
						mw->enable_menu_item(true, 1, mw->get_ui()->menuall_Preset_Positions);
						mw->enable_menu_item(true, 1, mw->get_ui()->actionall_Synchronisation);

						break;
					case UI_ALL_EDPS_ALL_SYNCHRONISED:

						mw->enable_menu_item(true, 1, mw->get_ui()->menuall_Preset_Positions);

						break;
					default:
						break;
				}

				break;

			case UI_ALL_EDPS_ALL_LOADED:
				mw->get_ui()->label_all_edps_notification->setText("ALL_LOADED		");
				mw->enable_menu_item(true, 2, mw->get_ui()->menuRobot, mw->get_ui()->menuAll_Robots);

				mw->enable_menu_item(false, 1, mw->get_ui()->actionall_EDP_Load);

				switch (all_edps_synchro)
				{
					case UI_ALL_EDPS_SYNCHRO_STATE_NOT_KNOWN:
					case UI_ALL_EDPS_NONE_SYNCHRONISED:
						mw->enable_menu_item(true, 1, mw->get_ui()->actionall_EDP_Unload);
						mw->enable_menu_item(false, 1, mw->get_ui()->menuall_Preset_Positions);
						mw->enable_menu_item(true, 1, mw->get_ui()->actionall_Synchronisation);

						break;
					case UI_ALL_EDPS_SOME_SYNCHRONISED:
						mw->enable_menu_item(true, 1, mw->get_ui()->actionall_EDP_Unload);
						mw->enable_menu_item(true, 1, mw->get_ui()->menuall_Preset_Positions);
						mw->enable_menu_item(true, 1, mw->get_ui()->actionall_Synchronisation);

						break;
					case UI_ALL_EDPS_ALL_SYNCHRONISED:

						mw->enable_menu_item(true, 1, mw->get_ui()->menuall_Preset_Positions);

						switch (mp.state)
						{
							case common::UI_MP_NOT_PERMITED_TO_RUN:
								mw->enable_menu_item(true, 2, mw->get_ui()->actionall_EDP_Unload, mw->get_ui()->menuall_Preset_Positions);
								break;
							case common::UI_MP_PERMITED_TO_RUN:

								mw->enable_menu_item(true, 2, mw->get_ui()->actionall_EDP_Unload, mw->get_ui()->menuall_Preset_Positions);
								break;
							case common::UI_MP_WAITING_FOR_START_PULSE:

								mw->enable_menu_item(false, 1, mw->get_ui()->actionall_EDP_Unload);
								mw->enable_menu_item(true, 1, mw->get_ui()->menuall_Preset_Positions);
								break;
							case common::UI_MP_TASK_RUNNING:
							case common::UI_MP_TASK_PAUSED:

								mw->enable_menu_item(false, 1, mw->get_ui()->actionall_EDP_Unload);
								mw->enable_menu_item(false, 1, mw->get_ui()->menuall_Preset_Positions);
								break;
							default:
								break;
						}
						break;
					default:
						break;
				}
				break;
			default:
				break;
		}
		all_edps_last_manage_interface_state = all_edps;
		all_edps_synchro_last_manage_interface_state = all_edps_synchro;
	}

	if (mp.state != mp.last_manage_interface_state) {
		// wlasciwosci menu task_menu
		switch (mp.state)
		{
			case common::UI_MP_NOT_PERMITED_TO_RUN:
				mw->get_ui()->label_mp_notification->setText("NOT_PERMITED_TO_RUN");
				mw->enable_menu_item(false, 2, mw->get_ui()->actionMP_Load, mw->get_ui()->actionMP_Unload);

				break;
			case common::UI_MP_PERMITED_TO_RUN:
				mw->get_ui()->label_mp_notification->setText("PERMITED_TO_RUN");
				mw->enable_menu_item(false, 1, mw->get_ui()->actionMP_Unload);
				mw->enable_menu_item(true, 1, mw->get_ui()->actionMP_Load);

				break;
			case common::UI_MP_WAITING_FOR_START_PULSE:
				mw->get_ui()->label_mp_notification->setText("WAITING_FOR_START_PULSE");
				mw->enable_menu_item(true, 1, mw->get_ui()->actionMP_Unload);
				mw->enable_menu_item(false, 2, mw->get_ui()->actionMP_Load, mw->get_ui()->actionall_EDP_Unload);

				break;
			case common::UI_MP_TASK_RUNNING:
				mw->get_ui()->label_mp_notification->setText("TASK_RUNNING");
				mw->enable_menu_item(false, 2, mw->get_ui()->actionMP_Load, mw->get_ui()->actionMP_Unload);
				break;
			case common::UI_MP_TASK_PAUSED:
				mw->get_ui()->label_mp_notification->setText("TASK_PAUSED");
				mw->enable_menu_item(false, 2, mw->get_ui()->actionMP_Load, mw->get_ui()->actionMP_Unload);

				break;
			default:
				break;
		}
		mp.last_manage_interface_state = mp.state;
	}

}

void Interface::reload_whole_configuration()
{

	if (access(config_file_relativepath.c_str(), R_OK) != 0) {
		std::cerr << "Wrong entry in default_file.cfg - load another configuration than: " << config_file_relativepath
				<< std::endl;
		config_file_relativepath = mrrocpp_bin_to_root_path + "configs/common.ini";
	}

	if ((mp.state == UI_MP_NOT_PERMITED_TO_RUN) || (mp.state == UI_MP_PERMITED_TO_RUN)) { // jesli nie dziala mp podmien mp ecp vsp

		config->change_config_file("../" + config_file);

		is_mp_and_ecps_active = config->value <int> ("is_active", "[mp]");

		switch (all_edps)
		{
			case UI_ALL_EDPS_NONE_ACTIVATED:
			case UI_ALL_EDPS_NONE_LOADED:

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

			if (!config->exists("node_name", lib::MP_SECTION)) {
				mp.node_name = "localhost";
			} else {
				mp.node_name = config->value <std::string> ("node_name", lib::MP_SECTION);
			}

			mp.pid = -1;
		}

		// inicjacja komunikacji z watkiem sr
		if (ui_msg == NULL) {
			ui_msg
					= (boost::shared_ptr <lib::sr_ui>) new lib::sr_ui(lib::UI, ui_attach_point.c_str(), network_sr_attach_point);
		}

		// inicjacja komunikacji z watkiem sr
		if (all_ecp_msg == NULL) {
			all_ecp_msg
					= (boost::shared_ptr <lib::sr_ecp>) new lib::sr_ecp(lib::ECP, "ui_all_ecp", network_sr_attach_point);
		}

		// wypisanie komunikatu o odczytaniu konfiguracji
		if (ui_msg) {
			std::string msg(config_file);
			msg += " config file loaded";
			ui_msg->message(msg.c_str());
		}
		mw->get_ui()->label_config_file_notification->setText(config_file.c_str());

	}

	manage_interface();
}

void Interface::UI_close(void)
{
	if (ui_state < 2) {
		printf("UI CLOSING\n");
		delay(100);// czas na ustabilizowanie sie edp
		ui_state = 2;// funcja OnTimer dowie sie ze aplikacja ma byc zamknieta
	}
}

void Interface::abort_threads()

{
	delete ui_sr_obj;
	delete ui_ecp_obj;

	delete meb_tid;
}

bool Interface::check_node_existence(const std::string & _node, const std::string & beginnig_of_message)
{
	/*
	 char buffer[50];
	 char c[20];
	 sprintf(buffer, "ping -c 3 %s | grep -c ms > a.txt", _node.c_str());
	 system(buffer);
	 FILE *p = fopen("a.txt", "r");
	 fgets(c, 5, p);
	 fclose(p);
	 if (strcmp(c, "5\n") != 0) {
	 std::string tmp(beginnig_of_message);
	 tmp += std::string(" node: ") + _node + std::string(" is unreachable");
	 ui_msg->message(lib::NON_FATAL_ERROR, tmp);

	 return false;
	 }

	 return true;
	 */
	/*
	 std::string opendir_path("/net/");
	 opendir_path += _node;

	 if (access(opendir_path.c_str(), R_OK) != 0) {
	 std::string tmp(beginnig_of_message);
	 tmp += std::string(" node: ") + _node + std::string(" is unreachable");
	 ui_msg->message(lib::NON_FATAL_ERROR, tmp);

	 return false;
	 }
	 return true;
	 */
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
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if (robot_node.second->state.is_active) {
						return true;
					}
				}

	return false;
}

bool Interface::are_all_active_robots_loaded()
{

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if ((robot_node.second->state.is_active) && (robot_node.second->state.edp.state <= 0)) {

						return false;
					}
				}

	return true;
}

bool Interface::is_any_active_robot_loaded()
{

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if ((robot_node.second->state.is_active) && (robot_node.second->state.edp.state > 0)) {
						return true;
					}
				}

	return false;
}

bool Interface::are_all_loaded_robots_synchronised()
{

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if ((robot_node.second->state.edp.state > 0) && (!(robot_node.second->state.edp.is_synchronised))) {

						return false;
					}
				}

	return true;
}

bool Interface::is_any_loaded_robot_synchronised()
{
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if ((robot_node.second->state.edp.state > 0) && (robot_node.second->state.edp.is_synchronised)) {
						return true;
					}
				}

	return false;
}

// ustala stan wszytkich EDP
int Interface::check_edps_state_and_modify_mp_state()
{

	// wyznaczenie stanu wszytkich EDP abstahujac od MP

	// jesli wszytkie sa nieaktywne
	if (!is_any_robot_active()) {
		all_edps = UI_ALL_EDPS_NONE_ACTIVATED;

	} else if (are_all_active_robots_loaded()) {
		all_edps = UI_ALL_EDPS_ALL_LOADED;

	} else if (is_any_active_robot_loaded()) {

		all_edps = UI_ALL_EDPS_SOME_LOADED;

		// jesli zaden nie jest zaladowany
	} else {
		all_edps = UI_ALL_EDPS_NONE_LOADED;

	}

	if ((all_edps == UI_ALL_EDPS_NONE_ACTIVATED) || (all_edps == UI_ALL_EDPS_NONE_LOADED)) {
		all_edps_synchro = UI_ALL_EDPS_SYNCHRO_NONE_EDP_LOADED;
	} else {

		// jesli wszytkie sa zsynchronizowane
		if (are_all_loaded_robots_synchronised()) {
			all_edps_synchro = UI_ALL_EDPS_ALL_SYNCHRONISED;

		} else if (is_any_loaded_robot_synchronised()) {
			all_edps_synchro = UI_ALL_EDPS_SOME_SYNCHRONISED;

		} else {

			all_edps_synchro = UI_ALL_EDPS_NONE_SYNCHRONISED;

		}
	}

	// modyfikacja stanu MP przez stan wszystkich EDP

	if ((all_edps == UI_ALL_EDPS_NONE_ACTIVATED) || ((all_edps == UI_ALL_EDPS_ALL_LOADED) && (all_edps_synchro
			== UI_ALL_EDPS_ALL_SYNCHRONISED))) {
		if ((mp.state == UI_MP_NOT_PERMITED_TO_RUN) && (is_mp_and_ecps_active)) {
			mp.state = UI_MP_PERMITED_TO_RUN; // pozwol na uruchomienie mp
		}
	} else {
		if (mp.state == UI_MP_PERMITED_TO_RUN) {
			mp.state = UI_MP_NOT_PERMITED_TO_RUN; // nie pozwol na uruchomienie mp
		}
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
		} else {
			outfile << config_file;
		}
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

		if (config->exists("program_name", *section_list_iterator)
				&& config->exists("is_active", *section_list_iterator)
				&& config->value <bool> ("is_active", *section_list_iterator)) {
			//	char* tmp_p =config->value<std::string>("program_name", *section_list_iterator);
			//	char* tmp_n =config->value<std::string>("node_name", *section_list_iterator);

			program_node_user_def tmp_s;

			tmp_s.program_name = config->value <std::string> ("program_name", *section_list_iterator);
			if (config->exists("node_name", *section_list_iterator)) {
				tmp_s.node_name = config->value <std::string> ("node_name", *section_list_iterator);
			} else {
				tmp_s.node_name = std::string("localhost");
			}

			if (config->exists("username", *section_list_iterator)) {
				tmp_s.user_name = config->value <std::string> ("username", *section_list_iterator);
			} else {
				tmp_s.user_name = getenv("USER");
			}

			if (config->exists("is_qnx", *section_list_iterator)) {
				tmp_s.is_qnx = config->value <bool> ("is_qnx", *section_list_iterator);
			} else {
				tmp_s.is_qnx = false;
			}

			program_node_user_list.push_back(tmp_s);
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
	program_node_user_list.clear();

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
		if (config) {
			delete config;
		}
		config = new lib::configurator(ui_node_name, mrrocpp_local_path, lib::UI_SECTION);

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

		if (messip::port_send_pulse(mp.pulse_fd, pulse_code, pulse_value))

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

	ui_ecp_obj = new ecp_buffer(*this);

	delay(1);
	ui_sr_obj = new sr_buffer(*this);

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

		if (mp.pulse_fd != lib::invalid_fd) {
			messip::port_disconnect(mp.pulse_fd);
		} else {
			std::cerr << "MP pulse not connected?" << std::endl;
		}

		// 	printf("dddd: %d\n", SignalKill(ini_con->mp-
		// 	printf("mp slay\n");

		//	SignalKill(mp.node_nr, mp.pid, 0, SIGTERM, 0, 0);

		if (kill(mp.pid, SIGTERM) == -1) {
			perror("kill()");
		} else {
			//    		int status;
			//    		if (waitpid(EDP_MASTER_Pid, &status, 0) == -1) {
			//    			perror("waitpid()");
			//    		}
		}

		mp.state = ui::common::UI_MP_PERMITED_TO_RUN; // mp wylaczone

	}
	// delay(1000);
	// 	kill(mp_pid,SIGTERM);
	// 	printf("mp pupa po kill\n");
	mp.pid = -1;
	mp.pulse_fd = lib::invalid_fd;
	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->deactivate_ecp_trigger();
				}

	// modyfikacja menu
	manage_interface();
	wgt_pc->process_control_window_init();
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

		wgt_pc->process_control_window_init();
		manage_interface();
	}

	return 1;

}

int Interface::pulse_stop_mp()

{

	if ((mp.state == ui::common::UI_MP_TASK_RUNNING) || (mp.state == ui::common::UI_MP_TASK_PAUSED)) {

		mp.state = ui::common::UI_MP_WAITING_FOR_START_PULSE;// czekanie na stop

		execute_mp_pulse(MP_STOP);

		manage_interface();
	}

	return 1;

}

int Interface::pulse_pause_mp()

{

	if (mp.state == ui::common::UI_MP_TASK_RUNNING) {

		mp.state = ui::common::UI_MP_TASK_PAUSED;// czekanie na stop

		execute_mp_pulse(MP_PAUSE);

		manage_interface();
	}

	return 1;

}

int Interface::pulse_resume_mp()

{

	if (mp.state == ui::common::UI_MP_TASK_PAUSED) {

		mp.state = ui::common::UI_MP_TASK_RUNNING;// czekanie na stop

		execute_mp_pulse(MP_RESUME);

		manage_interface();
	}

	return 1;

}

int Interface::pulse_trigger_mp()

{

	if (mp.state == ui::common::UI_MP_TASK_RUNNING) {

		execute_mp_pulse(MP_TRIGGER);

		manage_interface();
	}

	return 1;

}

//ECP pulse
int Interface::pulse_trigger_ecp()
{

	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->pulse_ecp();
				}

	return 1;
}

//Reader pulse
int Interface::pulse_start_all_reader()
{
	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->pulse_reader_start_exec_pulse();
				}

	manage_pc();

	return 1;
}

int Interface::pulse_stop_all_reader()
{
	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->pulse_reader_stop_exec_pulse();
				}
	manage_pc();
	return 1;
}

int Interface::pulse_trigger_all_reader()
{
	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->pulse_reader_trigger_exec_pulse();
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

	for (std::list <ui::common::program_node_user_def>::iterator program_node_user_list_iterator =
			program_node_user_list.begin(); program_node_user_list_iterator != program_node_user_list.end(); program_node_user_list_iterator++) {
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
		 */delay(100);

		if (program_node_user_list_iterator->is_qnx) {
			sprintf(system_command, "rsh -l %s %s slay -v -f %s", program_node_user_list_iterator->user_name.c_str(), program_node_user_list_iterator->node_name.c_str(), program_node_user_list_iterator->program_name.c_str());

		} else {
			sprintf(system_command, "rsh -l %s %s killall -e -q %s", program_node_user_list_iterator->user_name.c_str(), program_node_user_list_iterator->node_name.c_str(), program_node_user_list_iterator->program_name.c_str());

		}

		printf("slay_all: %s\n", system_command);
		// przedniolsem wywolanie system do innego prceosu bo w procesie glownym czasem powoduje zawieszenie calego intefrejsu
		pid_t child_pid = vfork();

		if (child_pid == 0) {
			system(system_command);
			_exit(EXIT_SUCCESS);
		} else if (child_pid > 0) {
			//delay(5000);
			printf("slay_all child %d created\n", child_pid);

		} else {
			perror("slay_all vfork()");
		}

	}
	printf("slay_all end\n");
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
