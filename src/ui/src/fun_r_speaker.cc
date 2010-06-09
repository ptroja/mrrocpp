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

#include "lib/srlib.h"
#include "ui/ui_const.h"
#include "ui/ui_class.h"
// #include "ui/ui.h"
// Konfigurator.
// #include "lib/configurator.h"
#include "lib/robot_consts/speaker_const.h"
#include "ui/src/speaker/ui_ecp_r_speaker.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern Ui ui;

// zamykanie okna odtwarzania dzwiekow dla robota speaker

int close_wnd_speaker_play(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	if (ui.speaker->is_wind_speaker_play_open) {
		PtDestroyWidget(ABW_wnd_speaker_play);
	}

	return (Pt_CONTINUE);

}

int start_wind_speaker_play(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!ui.speaker->is_wind_speaker_play_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_speaker_play, widget, cbinfo);
		ui.speaker->is_wind_speaker_play_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_speaker_play);
	}

	return (Pt_CONTINUE);

}

int clear_wind_speaker_play_flag(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui.speaker->is_wind_speaker_play_open = false;
	return (Pt_CONTINUE);

}

// inicjacja okna do odtwarzania dzwiekow dla robota speaker

int init_wnd_speaker_play(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	speaker_check_state(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int speaker_preset_sound_play(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	std::string text;
	std::string prosody;

	try {

		if (ui.speaker->state.edp.pid != -1) {

			if ((ApName(ApWidget(cbinfo)) == ABN_mm_speaker_preset_sound_0)
					|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
							== 0x30))) {
				text = ui.speaker->state.edp.preset_sound_0;
				prosody = "neutral";
			} else if ((ApName(ApWidget(cbinfo))
					== ABN_mm_speaker_preset_sound_1) || ((cbinfo->event->type
					== Ph_EV_KEY) && (my_data->key_cap == 0x31))) {
				text = ui.speaker->state.edp.preset_sound_1;
				prosody = "neutral";
			} else if ((ApName(ApWidget(cbinfo))
					== ABN_mm_speaker_preset_sound_2) || ((cbinfo->event->type
					== Ph_EV_KEY) && (my_data->key_cap == 0x32))) {
				text = ui.speaker->state.edp.preset_sound_2;
				prosody = "neutral";
			}

			ui.speaker->ui_ecp_robot->send_command(text.c_str(), prosody.c_str());

		}

	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

// funkcja wysylajaca rozkaz odtworzenia dzwieku do robota speaker

int speaker_play_exec(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char local_text[MAX_TEXT];
	char local_prosody[MAX_PROSODY];
	char* ref_local_text;
	char* ref_local_prosody;

	// wychwytania ew. bledow ECP::robot
	try {

		if (ui.speaker->state.edp.pid != -1)

			PtGetResource(ABW_PtText_wnd_speaker_play_text_entry,
					Pt_ARG_TEXT_STRING, &ref_local_text, 0);
		PtGetResource(ABW_PtComboBox_wnd_speaker_play_prosody_entry,
				Pt_ARG_TEXT_STRING, &ref_local_prosody, 0);
		strcpy(local_text, ref_local_text);
		strcpy(local_prosody, ref_local_prosody);

		ui.speaker->ui_ecp_robot->send_command(local_text, local_prosody);

		speaker_check_state(widget, apinfo, cbinfo);

	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);
}

int speaker_check_state(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if (ui.speaker->state.edp.pid != -1) {
			ui.speaker->ui_ecp_robot->read_state(
					&(ui.speaker->ui_ecp_robot->speaking_state));

			if (ui.speaker->ui_ecp_robot->speaking_state) { // odtwarzanie w toku
				PtSetResource(ABW_PtLabel_wnd_speaker_play_status,
						Pt_ARG_TEXT_STRING, "busy", 0);
			} else {
				PtSetResource(ABW_PtLabel_wnd_speaker_play_status,
						Pt_ARG_TEXT_STRING, "ready", 0);
			}

			PtDamageWidget(ABW_wnd_speaker_play);
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int EDP_speaker_create(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_PROCESS_CREATION);

	//char tmp_string[100];
	//char tmp2_string[100];

	try { // dla bledow robot :: ECP_error

		// dla robota speaker
		if (ui.speaker->state.edp.state == 0) {
			ui.speaker->state.edp.state = 0;
			ui.speaker->state.edp.is_synchronised = false;

			std::string tmp_string("/dev/name/global/");
			tmp_string += ui.speaker->state.edp.hardware_busy_attach_point;

			std::string tmp2_string("/dev/name/global/");
			tmp2_string
					+= ui.speaker->state.edp.network_resourceman_attach_point;

			// sprawdzeie czy nie jest juz zarejestrowany zarzadca zasobow
			if (((!(ui.speaker->state.edp.test_mode)) && (access(
					tmp_string.c_str(), R_OK) == 0)) || (access(
					tmp2_string.c_str(), R_OK) == 0)) {
				ui.ui_msg->message("edp_speaker already exists");

			} else if (ui.check_node_existence(ui.speaker->state.edp.node_name,
					std::string("edp_speaker"))) {

				ui.speaker->state.edp.node_nr = ui.config->return_node_number(
						ui.speaker->state.edp.node_name);

				ui.speaker->ui_ecp_robot = new ui_speaker_robot(
						&ui.speaker->state.edp, *ui.config, *ui.all_ecp_msg);
				ui.speaker->state.edp.pid
						= ui.speaker->ui_ecp_robot->get_EDP_pid();

				if (ui.speaker->state.edp.pid < 0) {
					ui.speaker->state.edp.state = 0;
					fprintf(stderr, "EDP spawn failed: %s\n", strerror(errno));
					delete ui.speaker->ui_ecp_robot;
				} else { // jesli spawn sie powiodl

					ui.speaker->state.edp.state = 1;

					/*
					 tmp = 0;
					 // kilka sekund  (~1) na otworzenie urzadzenia
					 while((ui.speaker->state.edp.reader_fd = name_open(ini_con->edp_speaker->network_reader_attach_point,
					 NAME_FLAG_ATTACH_GLOBAL))  < 0)
					 if((tmp++)<20)
					 delay(50);
					 else{
					 perror("blad odwolania do READER_START");
					 break;
					 };
					 */

					//ui.speaker->state.edp.state=1;// edp wlaczone reader czeka na start
					ui.speaker->state.edp.is_synchronised = true;
				}
			}
		}

	} // end try
	CATCH_SECTION_UI

	ui.manage_interface();

	return (Pt_CONTINUE);

}

int EDP_speaker_slay(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// dla robota speaker
	if (ui.speaker->state.edp.state > 0) { // jesli istnieje EDP
		if (ui.speaker->state.edp.reader_fd >= 0) {
			if (name_close(ui.speaker->state.edp.reader_fd) == -1) {
				fprintf(stderr, "UI: EDP_speaker, %s:%d, name_close(): %s\n",
						__FILE__, __LINE__, strerror(errno));
			}
		}

		delete ui.speaker->ui_ecp_robot;
		ui.speaker->state.edp.state = 0; // edp wylaczone
		ui.speaker->state.edp.is_synchronised = false;

		ui.speaker->state.edp.pid = -1;
		ui.speaker->state.edp.reader_fd = -1;

		close_wnd_speaker_play(NULL, NULL, NULL);

	}

	// modyfikacja menu
	ui.manage_interface();

	return (Pt_CONTINUE);

}

int pulse_reader_speaker_start(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.speaker->pulse_reader_start_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_reader_speaker_stop(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.speaker->pulse_reader_stop_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_reader_speaker_trigger(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.speaker->pulse_reader_trigger_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_ecp_speaker(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	char pulse_code = ECP_TRIGGER;
	long pulse_value = 1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.speaker->state.edp.is_synchronised > 0) { // o ile ECP dziala (sprawdzanie poprzez dzialanie odpowiedniego EDP)
		if (ui.speaker->state.ecp.trigger_fd < 0) {

			short tmp = 0;
			// kilka sekund  (~1) na otworzenie urzadzenia
			// zabezpieczenie przed zawieszeniem poprzez wyslanie sygnalu z opoznieniem
			ualarm((useconds_t) (SIGALRM_TIMEOUT), 0);
			while ((ui.speaker->state.ecp.trigger_fd = name_open(
					ui.speaker->state.ecp.network_trigger_attach_point.c_str(),
					NAME_FLAG_ATTACH_GLOBAL)) < 0) {
				if (errno == EINTR)
					break;
				if ((tmp++) < CONNECT_RETRY)
					delay(CONNECT_DELAY);
				else {
					perror("blad odwolania do ECP_TRIGGER");
				};
			}
			// odwolanie alarmu
			ualarm((useconds_t) (0), 0);
		}

		if (ui.speaker->state.ecp.trigger_fd >= 0) {
			if (MsgSendPulse(ui.speaker->state.ecp.trigger_fd,
					sched_get_priority_min(SCHED_FIFO), pulse_code, pulse_value)
					== -1) {

				fprintf(stderr, "Blad w wysylaniu pulsu do ecp error: %s \n",
						strerror(errno));
				delay(1000);
			}
		} else {
			printf("W PULS ECP:  BLAD name_open \n");
		}
	}

	return (Pt_CONTINUE);

}
