/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.03  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/netmgr.h>
#include <errno.h>
#include <process.h>
#include <math.h>

#include <boost/bind.hpp>

#include "lib/srlib.h"
#include "ui/ui_const.h"
// #include "ui/ui.h"
// Konfigurator.
#include "lib/configurator.h"
#include "ui/ui_ecp.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern boost::mutex process_creation_mtx;
extern function_execution_buffer edp_bird_hand_eb;
extern ui_state_def ui_state;
extern lib::configurator* config;
extern ui_msg_def ui_msg;
extern ui_robot_def ui_robot;

int EDP_bird_hand_create(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	edp_bird_hand_eb.command(boost::bind(EDP_bird_hand_create_int, widget,
			apinfo, cbinfo));

	return (Pt_CONTINUE);

}

int EDP_bird_hand_create_int(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	//	sleep(10);
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_PROCESS_CREATION);

	try { // dla bledow robot :: ECP_error

		// dla robota bird_hand
		if (ui_state.bird_hand.edp.state == 0) {

			ui_state.bird_hand.edp.state = 0;
			ui_state.bird_hand.edp.is_synchronised = false;

			std::string tmp_string("/dev/name/global/");
			tmp_string += ui_state.bird_hand.edp.hardware_busy_attach_point;

			std::string tmp2_string("/dev/name/global/");
			tmp2_string
					+= ui_state.bird_hand.edp.network_resourceman_attach_point;

			// sprawdzenie czy nie jest juz zarejestrowany zarzadca zasobow
			if ((!(ui_state.bird_hand.edp.test_mode)) && (access(
					tmp_string.c_str(), R_OK) == 0) || (access(
					tmp2_string.c_str(), R_OK) == 0)) {
				ui_msg.ui->message(lib::NON_FATAL_ERROR,
						"edp_bird_hand already exists");
			} else if (check_node_existence(ui_state.bird_hand.edp.node_name,
					"edp_bird_hand")) {

				ui_state.bird_hand.edp.node_nr = config->return_node_number(
						ui_state.bird_hand.edp.node_name);
				{
					boost::unique_lock<boost::mutex> lock(process_creation_mtx);
					ui_robot.bird_hand = new ui_tfg_and_conv_robot(*config,
							*ui_msg.all_ecp, lib::ROBOT_BIRD_HAND);

				}

				ui_state.bird_hand.edp.pid
						= ui_robot.bird_hand->ecp->get_EDP_pid();

				if (ui_state.bird_hand.edp.pid < 0) {

					ui_state.bird_hand.edp.state = 0;
					fprintf(stderr, "EDP spawn failed: %s\n", strerror(errno));
					delete ui_robot.bird_hand;
				} else { // jesli spawn sie powiodl

					ui_state.bird_hand.edp.state = 1;

					short tmp = 0;
					// kilka sekund  (~1) na otworzenie urzadzenia

					while ((ui_state.bird_hand.edp.reader_fd
							= name_open(
									ui_state.bird_hand.edp.network_reader_attach_point.c_str(),
									NAME_FLAG_ATTACH_GLOBAL)) < 0)
						if ((tmp++) < CONNECT_RETRY) {
							delay(CONNECT_DELAY);
						} else {
							perror("blad odwolania do READER_OT");
							break;
						}

					// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)
					lib::controller_state_t robot_controller_initial_state_tmp;

					ui_robot.bird_hand->get_controller_state(
							robot_controller_initial_state_tmp);

					//ui_state.bird_hand.edp.state = 1; // edp wlaczone reader czeka na start

					ui_state.bird_hand.edp.is_synchronised
							= robot_controller_initial_state_tmp.is_synchronised;
				}
			}
		}

	} // end try

	CATCH_SECTION_UI

	manage_interface();

	return 1;
}

int EDP_bird_hand_slay(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	edp_bird_hand_eb.command(boost::bind(EDP_bird_hand_slay_int, widget,
			apinfo, cbinfo));

	return (Pt_CONTINUE);

}

int EDP_bird_hand_slay_int(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{
	int pt_res;
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// dla robota bird_hand
	if (ui_state.bird_hand.edp.state > 0) { // jesli istnieje EDP
		if (ui_state.bird_hand.edp.reader_fd >= 0) {
			if (name_close(ui_state.bird_hand.edp.reader_fd) == -1) {
				fprintf(stderr, "UI: EDP_irp6ot, %s:%d, name_close(): %s\n",
						__FILE__, __LINE__, strerror(errno));
			}
		}
		delete ui_robot.bird_hand;
		ui_state.bird_hand.edp.state = 0; // edp wylaczone
		ui_state.bird_hand.edp.is_synchronised = false;

		ui_state.bird_hand.edp.pid = -1;
		ui_state.bird_hand.edp.reader_fd = -1;
		pt_res = PtEnter(0);
		close_all_irp6ot_windows(NULL, NULL, NULL);
		if (pt_res >= 0)
			PtLeave(0);
	}

	// modyfikacja menu

	manage_interface();

	return (Pt_CONTINUE);

}

int reload_bird_hand_configuration() {
	// jesli IRP6 on_track ma byc aktywne
	if ((ui_state.bird_hand.is_active = config->value<int> (
			"is_bird_hand_active")) == 1) {
		// ini_con->create_ecp_bird_hand (ini_con->ui->ecp_bird_hand_section);
		//ui_state.is_any_edp_active = true;
		if (ui_state.is_mp_and_ecps_active) {
			ui_state.bird_hand.ecp.network_trigger_attach_point
					= config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"trigger_attach_point",
							ui_state.bird_hand.ecp.section_name);

			ui_state.bird_hand.ecp.pid = -1;
			ui_state.bird_hand.ecp.trigger_fd = -1;
		}

		switch (ui_state.bird_hand.edp.state) {
		case -1:
		case 0:
			// ini_con->create_edp_bird_hand (ini_con->ui->edp_bird_hand_section);

			ui_state.bird_hand.edp.pid = -1;
			ui_state.bird_hand.edp.reader_fd = -1;
			ui_state.bird_hand.edp.state = 0;

			if (config->exists("test_mode", ui_state.bird_hand.edp.section_name))
				ui_state.bird_hand.edp.test_mode = config->value<int> (
						"test_mode", ui_state.bird_hand.edp.section_name);
			else
				ui_state.bird_hand.edp.test_mode = 0;

			ui_state.bird_hand.edp.hardware_busy_attach_point = config->value<
					std::string> ("hardware_busy_attach_point",
					ui_state.bird_hand.edp.section_name);

			ui_state.bird_hand.edp.network_resourceman_attach_point
					= config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"resourceman_attach_point",
							ui_state.bird_hand.edp.section_name);

			ui_state.bird_hand.edp.network_reader_attach_point
					= config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"reader_attach_point",
							ui_state.bird_hand.edp.section_name);

			ui_state.bird_hand.edp.node_name = config->value<std::string> (
					"node_name", ui_state.bird_hand.edp.section_name);
			break;
		case 1:
		case 2:
			// nie robi nic bo EDP pracuje
			break;
		default:
			break;
		}

	} else // jesli  irp6 on_track ma byc nieaktywne
	{
		switch (ui_state.bird_hand.edp.state) {
		case -1:
		case 0:
			ui_state.bird_hand.edp.state = -1;
			break;
		case 1:
		case 2:
			// nie robi nic bo EDP pracuje
			break;
		default:
			break;
		}
	} // end bird_hand

	return 1;
}

int manage_interface_bird_hand() {
	switch (ui_state.bird_hand.edp.state) {
	case -1:
		ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_bird_hand, NULL);
		break;
	case 0:
		ApModifyItemState(&robot_menu, AB_ITEM_DIM,
				ABN_mm_bird_hand_edp_unload,

				NULL);
		ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_bird_hand,
				ABN_mm_bird_hand_edp_load, NULL);

		break;
	case 1:
	case 2:
		ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_bird_hand, NULL);

		// jesli robot jest zsynchronizowany
		if (ui_state.bird_hand.edp.is_synchronised) {
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_preset_positions, NULL);

			switch (ui_state.mp.state) {
			case UI_MP_NOT_PERMITED_TO_RUN:
			case UI_MP_PERMITED_TO_RUN:
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,
						ABN_mm_bird_hand_edp_unload, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM,
						ABN_mm_bird_hand_edp_load, NULL);
				break;
			case UI_MP_WAITING_FOR_START_PULSE:
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,

				NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM,
						ABN_mm_bird_hand_edp_load, ABN_mm_bird_hand_edp_unload,
						NULL);
				break;
			case UI_MP_TASK_RUNNING:
			case UI_MP_TASK_PAUSED:
				ApModifyItemState(&robot_menu, AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
						NULL);
				break;
			default:
				break;
			}
		} else // jesli robot jest niezsynchronizowany
		{
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,
					ABN_mm_bird_hand_edp_unload, NULL);
			ApModifyItemState(&robot_menu, AB_ITEM_DIM,
					ABN_mm_bird_hand_edp_load, NULL);
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_synchronisation, NULL);
		}
		break;
	default:
		break;
	}

	return 1;
}

