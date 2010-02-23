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
// #include "ui/ui.h"
// Konfigurator.
#include "lib/configurator.h"
#include "ui/ui_ecp.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern function_execution_buffer edp_irp6ot_tfg_eb;
extern ui_state_def ui_state;
extern lib::configurator* config;
extern ui_msg_def ui_msg;
extern ui_robot_def ui_robot;

int EDP_irp6ot_tfg_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	edp_irp6ot_tfg_eb.command(boost::bind(EDP_irp6ot_tfg_create_int, widget, apinfo, cbinfo));

	return (Pt_CONTINUE);

}

int EDP_irp6ot_tfg_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	//	sleep(10);
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_PROCESS_CREATION);

	try { // dla bledow robot :: ECP_error

		// dla robota irp6ot_tfg
		if (ui_state.irp6ot_tfg.edp.state == 0) {

			ui_state.irp6ot_tfg.edp.state = 0;
			ui_state.irp6ot_tfg.edp.is_synchronised = false;

			std::string tmp_string("/dev/name/global/");
			tmp_string += ui_state.irp6ot_tfg.edp.hardware_busy_attach_point;

			std::string tmp2_string("/dev/name/global/");
			tmp2_string += ui_state.irp6ot_tfg.edp.network_resourceman_attach_point;

			// sprawdzenie czy nie jest juz zarejestrowany zarzadca zasobow
			if ((!(ui_state.irp6ot_tfg.edp.test_mode)) && (access(tmp_string.c_str(), R_OK) == 0)
					|| (access(tmp2_string.c_str(), R_OK) == 0)) {
				ui_msg.ui->message(lib::NON_FATAL_ERROR, "edp_irp6ot_tfg already exists");
			} else if (check_node_existence(ui_state.irp6ot_tfg.edp.node_name, std::string("edp_irp6ot_tfg"))) {

				ui_state.irp6ot_tfg.edp.node_nr = config->return_node_number(ui_state.irp6ot_tfg.edp.node_name);

				ui_state.irp6ot_tfg.edp.state = 1;

				ui_robot.irp6ot_tfg = new ui_tfg_robot(*config, *ui_msg.all_ecp, lib::ROBOT_IRP6OT_TFG);

				ui_state.irp6ot_tfg.edp.pid = ui_robot.irp6ot_tfg->ecp->get_EDP_pid();

				if (ui_state.irp6ot_tfg.edp.pid < 0) {

					ui_state.irp6ot_tfg.edp.state = 0;
					fprintf(stderr, "EDP spawn failed: %s\n", strerror(errno));
					delete ui_robot.irp6ot_tfg;
				} else { // jesli spawn sie powiodl

					short tmp = 0;
					// kilka sekund  (~1) na otworzenie urzadzenia

					while ((ui_state.irp6ot_tfg.edp.reader_fd
							= name_open(ui_state.irp6ot_tfg.edp.network_reader_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL))
							< 0)
						if ((tmp++) < CONNECT_RETRY) {
							delay(CONNECT_DELAY);
						} else {
							perror("blad odwolania do READER_OT");
							break;
						}

					// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)
					lib::controller_state_t robot_controller_initial_state_tmp;

					ui_robot.irp6ot_tfg->get_controller_state(robot_controller_initial_state_tmp);

					//ui_state.irp6ot_tfg.edp.state = 1; // edp wlaczone reader czeka na start

					ui_state.irp6ot_tfg.edp.is_synchronised = robot_controller_initial_state_tmp.is_synchronised;
				}
			}
		}

	} // end try

	CATCH_SECTION_UI

	manage_interface();

	return 1;
}

int EDP_irp6ot_tfg_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return (Pt_CONTINUE);

}

int EDP_irp6ot_tfg_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return (Pt_CONTINUE);

}

int start_wind_irp6ot_tfg_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return (Pt_CONTINUE);

}

int irp6ot_tfg_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return (Pt_CONTINUE);

}

int start_wnd_irp6ot_tfg_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return (Pt_CONTINUE);

}

int init_wnd_irp6ot_tfg_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return (Pt_CONTINUE);

}

int irp6ot_tfg_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return (Pt_CONTINUE);

}

int wind_irp6ot_tfg_moves_init(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return (Pt_CONTINUE);

}

int clear_wind_irp6ot_tfg_moves_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return (Pt_CONTINUE);

}

int wind_irp6ot_tfg_moves_move(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return (Pt_CONTINUE);

}

int clear_wnd_irp6ot_tfg_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return (Pt_CONTINUE);

}

int reload_irp6ot_tfg_configuration()
{
	// jesli IRP6 on_track ma byc aktywne
	if ((ui_state.irp6ot_tfg.is_active = config->value <int> ("is_irp6ot_tfg_active")) == 1) {
		// ini_con->create_ecp_irp6ot_tfg (ini_con->ui->ecp_irp6ot_tfg_section);
		//ui_state.is_any_edp_active = true;
		if (ui_state.is_mp_and_ecps_active) {
			ui_state.irp6ot_tfg.ecp.network_trigger_attach_point
					= config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "trigger_attach_point", ui_state.irp6ot_tfg.ecp.section_name);

			ui_state.irp6ot_tfg.ecp.pid = -1;
			ui_state.irp6ot_tfg.ecp.trigger_fd = -1;
		}

		switch (ui_state.irp6ot_tfg.edp.state)
		{
			case -1:
			case 0:
				// ini_con->create_edp_irp6ot_tfg (ini_con->ui->edp_irp6ot_tfg_section);

				ui_state.irp6ot_tfg.edp.pid = -1;
				ui_state.irp6ot_tfg.edp.reader_fd = -1;
				ui_state.irp6ot_tfg.edp.state = 0;

				for (int i = 0; i < 3; i++) {
					char tmp_string[50];
					sprintf(tmp_string, "preset_position_%d", i);

					if (config->exists(tmp_string, ui_state.irp6ot_tfg.edp.section_name)) {
						char* tmp, *tmp1;
						tmp1
								= tmp
										= strdup(config->value <std::string> (tmp_string, ui_state.irp6ot_tfg.edp.section_name).c_str());
						char* toDel = tmp;
						for (int j = 0; j < IRP6OT_TFG_NUM_OF_SERVOS; j++) {

							ui_state.irp6ot_tfg.edp.preset_position[i][j] = strtod(tmp1, &tmp1);

						}
						free(toDel);
					} else {
						for (int j = 0; j < IRP6OT_TFG_NUM_OF_SERVOS; j++) {

							ui_state.irp6ot_tfg.edp.preset_position[i][j] = 0.0;

						}
					}
				}

				if (config->exists("test_mode", ui_state.irp6ot_tfg.edp.section_name))
					ui_state.irp6ot_tfg.edp.test_mode
							= config->value <int> ("test_mode", ui_state.irp6ot_tfg.edp.section_name);
				else
					ui_state.irp6ot_tfg.edp.test_mode = 0;

				ui_state.irp6ot_tfg.edp.hardware_busy_attach_point
						= config->value <std::string> ("hardware_busy_attach_point", ui_state.irp6ot_tfg.edp.section_name);

				ui_state.irp6ot_tfg.edp.network_resourceman_attach_point
						= config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "resourceman_attach_point", ui_state.irp6ot_tfg.edp.section_name);

				ui_state.irp6ot_tfg.edp.network_reader_attach_point
						= config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "reader_attach_point", ui_state.irp6ot_tfg.edp.section_name);

				ui_state.irp6ot_tfg.edp.node_name
						= config->value <std::string> ("node_name", ui_state.irp6ot_tfg.edp.section_name);
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
		switch (ui_state.irp6ot_tfg.edp.state)
		{
			case -1:
			case 0:
				ui_state.irp6ot_tfg.edp.state = -1;
				break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
				break;
			default:
				break;
		}
	} // end irp6ot_tfg

	return 1;
}

