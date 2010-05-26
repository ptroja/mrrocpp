/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/ui_class.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

//
//
// KLASA ui
//
//


Ui::Ui() :
	config(NULL), all_ecp_msg(NULL), ui_msg(NULL),
			is_mp_and_ecps_active(false), all_edps(UI_ALL_EDPS_NONE_EDP_LOADED) {

	mp.state = UI_MP_NOT_PERMITED_TO_RUN;// mp wylaczone
	mp.last_state = UI_MP_NOT_PERMITED_TO_RUN;// mp wylaczone
	mp.pid = -1;

}

void Ui::init() {

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


}

// funkcja odpowiedzialna za wyglad aplikacji na podstawie jej stanu

int Ui::manage_interface(void) {
	int pt_res;
	pt_res = PtEnter(0);

	check_edps_state_and_modify_mp_state();

	// na wstepie wylaczamy przyciski EDP z all robots menu. Sa one ewentualnie wlaczane dalej
	ApModifyItemState(&all_robots_menu, AB_ITEM_DIM,
			ABN_mm_all_robots_preset_positions,
			ABN_mm_all_robots_synchronisation, ABN_mm_all_robots_edp_unload,
			ABN_mm_all_robots_edp_load, NULL);

	// menu file
	// ApModifyItemState( &file_menu, AB_ITEM_DIM, NULL);

	// Dla robota IRP6 ON_TRACK
	manage_interface_irp6ot();

	// Dla robota IRP6 ON_TRACK
	manage_interface_irp6ot_tfg();

	manage_interface_irp6p_tfg();

	// Dla robota IRP6 POSTUMENT
	manage_interface_irp6p();

	// Dla robota CONVEYOR
	manage_interface_conveyor();

	manage_interface_spkm();
	manage_interface_smb();
	manage_interface_shead();

	bird_hand.manage_interface();

	// Dla robota SPEAKER
	manage_interface_speaker();

	// Dla robota IRP6 MECHATRONIKA
	manage_interface_irp6m();

	// zadanie
	// kolorowanie menu all robots


	// wlasciwosci menu  ABW_base_all_robots


	switch (all_edps) {
	case UI_ALL_EDPS_NONE_EDP_ACTIVATED:
		//				printf("UI_ALL_EDPS_NONE_EDP_ACTIVATED\n");
		block_widget(ABW_base_all_robots);
		PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_GRAY, 0);
		block_widget(ABW_base_robot);
		PtSetResource(ABW_base_robot, Pt_ARG_COLOR, Pg_GRAY, 0);
		break;
	case UI_ALL_EDPS_NONE_EDP_LOADED:
		//				printf("UI_ALL_EDPS_NONE_EDP_LOADED\n");
		ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
				ABN_mm_all_robots_edp_load, NULL);
		PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_BLACK, 0);
		PtSetResource(ABW_base_robot, Pt_ARG_COLOR, Pg_BLACK, 0);
		unblock_widget(ABW_base_all_robots);
		unblock_widget(ABW_base_robot);
		break;
	case UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED:
		//			printf("UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED\n");
		ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
				ABN_mm_all_robots_edp_unload, NULL);
		ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
				ABN_mm_all_robots_edp_load, NULL);
		PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_BLACK, 0);
		PtSetResource(ABW_base_robot, Pt_ARG_COLOR, Pg_BLACK, 0);
		unblock_widget(ABW_base_all_robots);
		unblock_widget(ABW_base_robot);
		break;
	case UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED:
		//			printf("UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED\n");
		ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
				ABN_mm_all_robots_edp_unload, NULL);
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
		switch (mp.state) {
		case UI_MP_NOT_PERMITED_TO_RUN:
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_edp_unload, NULL);
			break;
		case UI_MP_PERMITED_TO_RUN:
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_edp_unload,
					ABN_mm_all_robots_preset_positions, NULL);
			break;
		case UI_MP_WAITING_FOR_START_PULSE:
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_preset_positions, NULL);
			break;
		case UI_MP_TASK_RUNNING:
		case UI_MP_TASK_PAUSED:
			ApModifyItemState(&all_robots_menu, AB_ITEM_DIM,
					ABN_mm_all_robots_preset_positions, NULL);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	// wlasciwosci menu task_menu
	switch (mp.state) {

	case UI_MP_NOT_PERMITED_TO_RUN:
		ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_load,
				ABN_mm_mp_unload, NULL);
		PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_BLACK, 0);
		break;
	case UI_MP_PERMITED_TO_RUN:
		ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_unload, NULL);
		ApModifyItemState(&task_menu, AB_ITEM_NORMAL, ABN_mm_mp_load, NULL);
		PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_BLACK, 0);
		break;
	case UI_MP_WAITING_FOR_START_PULSE:
		ApModifyItemState(&task_menu, AB_ITEM_NORMAL, ABN_mm_mp_unload, NULL);
		ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_load, NULL);
		//	ApModifyItemState( &all_robots_menu, AB_ITEM_DIM, ABN_mm_all_robots_edp_unload, NULL);
		PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_DBLUE, 0);
		break;
	case UI_MP_TASK_RUNNING:
	case UI_MP_TASK_PAUSED:
		ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_unload,
				ABN_mm_mp_load, NULL);
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

int Ui::reload_whole_configuration() {

	if (access(config_file_relativepath.c_str(), R_OK) != 0) {
		fprintf(
				stderr,
				"Wrong entry in default_file.cfg - load another configuration than: %s\n",
				config_file_relativepath.c_str());
		config_file_relativepath = "../configs/common.ini";
	}

	if ((mp.state == UI_MP_NOT_PERMITED_TO_RUN) || (mp.state
			== UI_MP_PERMITED_TO_RUN)) { // jesli nie dziala mp podmien mp ecp vsp

		config->change_ini_file(config_file.c_str());

		is_mp_and_ecps_active = config->value<int> ("is_mp_and_ecps_active");

		switch (all_edps) {
		case UI_ALL_EDPS_NONE_EDP_ACTIVATED:
		case UI_ALL_EDPS_NONE_EDP_LOADED:

			// dla robota irp6 on_track
			reload_irp6ot_configuration();

			// dla robota irp6 on_track
			reload_irp6ot_tfg_configuration();

			reload_irp6p_tfg_configuration();

			// dla robota irp6 postument
			reload_irp6p_configuration();

			// dla robota conveyor
			reload_conveyor_configuration();

			reload_spkm_configuration();
			reload_smb_configuration();
			reload_shead_configuration();

			bird_hand.reload_configuration();

			// dla robota speaker
			reload_speaker_configuration();

			// dla robota irp6 mechatronika
			reload_irp6m_configuration();
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
		 for (list<char*>::iterator list_iterator = ui.section_list.begin(); list_iterator != ui.section_list.end(); list_iterator++)
		 {
		 printf("section_name: %s\n", *list_iterator);

		 }

		 for (list<char*>::iterator node_list_iterator = ui_state.node_list.begin(); node_list_iterator != ui_state.node_list.end(); node_list_iterator++)
		 {
		 printf("node_name: %s\n", *node_list_iterator);
		 }

		 for (list<program_node_def>::iterator program_node_list_iterator = ui.program_node_list.begin(); program_node_list_iterator != ui.program_node_list.end(); program_node_list_iterator++)
		 {
		 printf("node_name: %s\n", program_node_list_iterator->node_name);
		 }
		 */

		// zczytanie konfiguracji UI


		// zczytanie konfiguracji MP

		if (is_mp_and_ecps_active) {
			mp.network_pulse_attach_point = config->return_attach_point_name(
					lib::configurator::CONFIG_SERVER, "mp_pulse_attach_point",
					MP_SECTION);
			mp.node_name = config->value<std::string> ("node_name", MP_SECTION);
			mp.pid = -1;
		}

		// inicjacja komunikacji z watkiem sr
		if (ui_msg == NULL) {
			ui_msg = new lib::sr_ui(lib::UI, ui_attach_point.c_str(),
					network_sr_attach_point.c_str(), false);
		}

		// inicjacja komunikacji z watkiem sr
		if (all_ecp_msg == NULL) {
			all_ecp_msg = new lib::sr_ecp(lib::ECP, "ui_all_ecp",
					network_sr_attach_point.c_str(), false);
		}

		// wypisanie komunikatu o odczytaniu konfiguracji
		if (ui_msg) {
			std::string msg(config_file);
			msg += " config file loaded";
			ui_msg->message(msg.c_str());
		}

	}

	manage_interface();

	return 1;
}

