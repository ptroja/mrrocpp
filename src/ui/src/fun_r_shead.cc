/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

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
#include "ui/ui_class.h"
// #include "ui/ui.h"
// Konfigurator.
#include "lib/configurator.h"
#include "ui/ui_ecp.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern Ui ui;

extern function_execution_buffer edp_shead_eb;
extern ui_state_def ui_state;

extern ui_msg_def ui_msg;
extern ui_robot_def ui_robot;
extern boost::mutex process_creation_mtx;

int EDP_shead_create(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	edp_shead_eb.command(boost::bind(EDP_shead_create_int, widget, apinfo,
			cbinfo));

	return (Pt_CONTINUE);

}

int EDP_shead_create_int(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	//	sleep(10);
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_PROCESS_CREATION);

	try { // dla bledow robot :: ECP_error

		// dla robota shead
		if (ui_state.shead.edp.state == 0) {

			ui_state.shead.edp.state = 0;
			ui_state.shead.edp.is_synchronised = false;

			std::string tmp_string("/dev/name/global/");
			tmp_string += ui_state.shead.edp.hardware_busy_attach_point;

			std::string tmp2_string("/dev/name/global/");
			tmp2_string += ui_state.shead.edp.network_resourceman_attach_point;

			// sprawdzenie czy nie jest juz zarejestrowany zarzadca zasobow
			if ((!(ui_state.shead.edp.test_mode)) && (access(
					tmp_string.c_str(), R_OK) == 0) || (access(
					tmp2_string.c_str(), R_OK) == 0)) {
				ui_msg.ui->message(lib::NON_FATAL_ERROR,
						"edp_shead already exists");
			} else if (check_node_existence(ui_state.shead.edp.node_name,
					std::string("edp_shead"))) {

				ui_state.shead.edp.node_nr = ui.config->return_node_number(
						ui_state.shead.edp.node_name);

				{
					boost::unique_lock<boost::mutex> lock(process_creation_mtx);
					ui_robot.shead = new ui_tfg_and_conv_robot(*ui.config,
							*ui_msg.all_ecp, lib::ROBOT_SHEAD);
				}
				ui_state.shead.edp.pid = ui_robot.shead->ecp->get_EDP_pid();

				if (ui_state.shead.edp.pid < 0) {

					ui_state.shead.edp.state = 0;
					fprintf(stderr, "EDP spawn failed: %s\n", strerror(errno));
					delete ui_robot.shead;
				} else { // jesli spawn sie powiodl
					ui_state.shead.edp.state = 1;
					short tmp = 0;
					// kilka sekund  (~1) na otworzenie urzadzenia

					while ((ui_state.shead.edp.reader_fd
							= name_open(
									ui_state.shead.edp.network_reader_attach_point.c_str(),
									NAME_FLAG_ATTACH_GLOBAL)) < 0)
						if ((tmp++) < CONNECT_RETRY) {
							delay(CONNECT_DELAY);
						} else {
							perror("blad odwolania do READER_OT");
							break;
						}

					// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)
					lib::controller_state_t robot_controller_initial_state_tmp;

					ui_robot.shead->get_controller_state(
							robot_controller_initial_state_tmp);

					//ui_state.shead.edp.state = 1; // edp wlaczone reader czeka na start

					ui_state.shead.edp.is_synchronised
							= robot_controller_initial_state_tmp.is_synchronised;
				}
			}
		}

	} // end try

	CATCH_SECTION_UI

	manage_interface();

	return 1;
}

int EDP_shead_slay(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	edp_shead_eb.command(
			boost::bind(EDP_shead_slay_int, widget, apinfo, cbinfo));

	return (Pt_CONTINUE);

}

int EDP_shead_slay_int(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{
	int pt_res;
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// dla robota shead
	if (ui_state.shead.edp.state > 0) { // jesli istnieje EDP
		if (ui_state.shead.edp.reader_fd >= 0) {
			if (name_close(ui_state.shead.edp.reader_fd) == -1) {
				fprintf(stderr, "UI: EDP_irp6ot, %s:%d, name_close(): %s\n",
						__FILE__, __LINE__, strerror(errno));
			}
		}
		delete ui_robot.shead;
		ui_state.shead.edp.state = 0; // edp wylaczone
		ui_state.shead.edp.is_synchronised = false;

		ui_state.shead.edp.pid = -1;
		ui_state.shead.edp.reader_fd = -1;
		pt_res = PtEnter(0);
		close_all_irp6ot_windows(NULL, NULL, NULL);
		if (pt_res >= 0)
			PtLeave(0);
	}

	// modyfikacja menu

	manage_interface();

	return (Pt_CONTINUE);

}

int reload_shead_configuration() {
	// jesli IRP6 on_track ma byc aktywne
	if ((ui_state.shead.is_active = ui.config->value<int> ("is_shead_active"))
			== 1) {
		// ini_con->create_ecp_shead (ini_con->ui->ecp_shead_section);
		//ui_state.is_any_edp_active = true;
		if (ui_state.is_mp_and_ecps_active) {
			ui_state.shead.ecp.network_trigger_attach_point
					= ui.config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"trigger_attach_point",
							ui_state.shead.ecp.section_name);

			ui_state.shead.ecp.pid = -1;
			ui_state.shead.ecp.trigger_fd = -1;
		}

		switch (ui_state.shead.edp.state) {
		case -1:
		case 0:
			// ini_con->create_edp_shead (ini_con->ui->edp_shead_section);

			ui_state.shead.edp.pid = -1;
			ui_state.shead.edp.reader_fd = -1;
			ui_state.shead.edp.state = 0;

			if (ui.config->exists("test_mode", ui_state.shead.edp.section_name))
				ui_state.shead.edp.test_mode = ui.config->value<int> (
						"test_mode", ui_state.shead.edp.section_name);
			else
				ui_state.shead.edp.test_mode = 0;

			ui_state.shead.edp.hardware_busy_attach_point = ui.config->value<
					std::string> ("hardware_busy_attach_point",
					ui_state.shead.edp.section_name);

			ui_state.shead.edp.network_resourceman_attach_point
					= ui.config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"resourceman_attach_point",
							ui_state.shead.edp.section_name);

			ui_state.shead.edp.network_reader_attach_point
					= ui.config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"reader_attach_point",
							ui_state.shead.edp.section_name);

			ui_state.shead.edp.node_name = ui.config->value<std::string> (
					"node_name", ui_state.shead.edp.section_name);
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
		switch (ui_state.shead.edp.state) {
		case -1:
		case 0:
			ui_state.shead.edp.state = -1;
			break;
		case 1:
		case 2:
			// nie robi nic bo EDP pracuje
			break;
		default:
			break;
		}
	} // end shead

	return 1;
}

int manage_interface_shead() {
	switch (ui_state.shead.edp.state) {
	case -1:
		ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_shead, NULL);
		break;
	case 0:
		ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_shead_edp_unload,

		NULL);
		ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_shead,
				ABN_mm_shead_edp_load, NULL);

		break;
	case 1:
	case 2:
		ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_shead, NULL);

		// jesli robot jest zsynchronizowany
		if (ui_state.shead.edp.is_synchronised) {
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_preset_positions, NULL);

			switch (ui_state.mp.state) {
			case UI_MP_NOT_PERMITED_TO_RUN:
			case UI_MP_PERMITED_TO_RUN:
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,
						ABN_mm_shead_edp_unload, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM,
						ABN_mm_shead_edp_load, NULL);
				break;
			case UI_MP_WAITING_FOR_START_PULSE:
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,

				NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM,
						ABN_mm_shead_edp_load, ABN_mm_shead_edp_unload, NULL);
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
					ABN_mm_shead_edp_unload, NULL);
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_shead_edp_load,
					NULL);
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_synchronisation, NULL);
		}
		break;
	default:
		break;
	}

	return 1;
}

