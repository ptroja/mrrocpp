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
#include "ui/ui_class.h"
// #include "ui/ui.h"
// Konfigurator.
#include "lib/configurator.h"
#include "ui/src/bird_hand/ui_ecp_r_bird_hand.h"
#include "lib/robot_consts/bird_hand_const.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern Ui ui;

int EDP_bird_hand_create(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui.bird_hand.eb.command(boost::bind(EDP_bird_hand_create_int, widget,
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
		if (ui.bird_hand.state.edp.state == 0) {

			ui.bird_hand.state.edp.state = 0;
			ui.bird_hand.state.edp.is_synchronised = false;

			std::string tmp_string("/dev/name/global/");
			tmp_string += ui.bird_hand.state.edp.hardware_busy_attach_point;

			std::string tmp2_string("/dev/name/global/");
			tmp2_string
					+= ui.bird_hand.state.edp.network_resourceman_attach_point;

			// sprawdzenie czy nie jest juz zarejestrowany zarzadca zasobow
			if (((!(ui.bird_hand.state.edp.test_mode)) && (access(
					tmp_string.c_str(), R_OK) == 0)) || (access(
					tmp2_string.c_str(), R_OK) == 0)) {
				ui.ui_msg->message(lib::NON_FATAL_ERROR,
						"edp_bird_hand already exists");
			} else if (ui.check_node_existence(ui.bird_hand.state.edp.node_name,
					std::string("edp_bird_hand"))) {

				ui.bird_hand.state.edp.node_nr = ui.config->return_node_number(
						ui.bird_hand.state.edp.node_name);
				{
					boost::unique_lock<boost::mutex> lock(
							ui.process_creation_mtx);
					ui.bird_hand.ui_ecp_robot = new ui_bird_hand_robot(
							*ui.config, *ui.all_ecp_msg);

				}

				ui.bird_hand.state.edp.pid
						= ui.bird_hand.ui_ecp_robot->the_robot->get_EDP_pid();

				if (ui.bird_hand.state.edp.pid < 0) {

					ui.bird_hand.state.edp.state = 0;
					fprintf(stderr, "EDP spawn failed: %s\n", strerror(errno));
					delete ui.bird_hand.ui_ecp_robot;
				} else { // jesli spawn sie powiodl

					ui.bird_hand.state.edp.state = 1;

					short tmp = 0;
					// kilka sekund  (~1) na otworzenie urzadzenia

					while ((ui.bird_hand.state.edp.reader_fd
							= name_open(
									ui.bird_hand.state.edp.network_reader_attach_point.c_str(),
									NAME_FLAG_ATTACH_GLOBAL)) < 0)
						if ((tmp++) < CONNECT_RETRY) {
							delay(CONNECT_DELAY);
						} else {
							perror("blad odwolania do READER_OT");
							break;
						}

					// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)
					lib::controller_state_t robot_controller_initial_state_tmp;

					ui.bird_hand.ui_ecp_robot->get_controller_state(
							robot_controller_initial_state_tmp);

					//ui.bird_hand.state.edp.state = 1; // edp wlaczone reader czeka na start

					ui.bird_hand.state.edp.is_synchronised
							= robot_controller_initial_state_tmp.is_synchronised;
				}
			}
		}

	} // end try

	CATCH_SECTION_UI

	ui.manage_interface();

	return 1;
}

int EDP_bird_hand_slay(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui.bird_hand.eb.command(boost::bind(EDP_bird_hand_slay_int, widget, apinfo,
			cbinfo));

	return (Pt_CONTINUE);

}

int EDP_bird_hand_slay_int(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{
	int pt_res;
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// dla robota bird_hand
	if (ui.bird_hand.state.edp.state > 0) { // jesli istnieje EDP
		if (ui.bird_hand.state.edp.reader_fd >= 0) {
			if (name_close(ui.bird_hand.state.edp.reader_fd) == -1) {
				fprintf(stderr, "UI: EDP_irp6ot, %s:%d, name_close(): %s\n",
						__FILE__, __LINE__, strerror(errno));
			}
		}
		delete ui.bird_hand.ui_ecp_robot;
		ui.bird_hand.state.edp.state = 0; // edp wylaczone
		ui.bird_hand.state.edp.is_synchronised = false;

		ui.bird_hand.state.edp.pid = -1;
		ui.bird_hand.state.edp.reader_fd = -1;
		pt_res = PtEnter(0);
		close_all_irp6ot_windows(NULL, NULL, NULL);
		if (pt_res >= 0)
			PtLeave(0);
	}

	// modyfikacja menu

	ui.manage_interface();

	return (Pt_CONTINUE);

}

