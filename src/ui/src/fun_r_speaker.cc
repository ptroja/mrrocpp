/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
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
#include <process.h>
#include <cmath>

#include "base/lib/sr/srlib.h"

#include "ui/src/ui_class.h"
// #include "ui/src/ui.h"
// Konfigurator.
// #include "base/lib/configurator.h"
#include "robot/speaker/const_speaker.h"
#include "ui/src/speaker/ui_ecp_r_speaker.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern ui::common::Interface interface;

// zamykanie okna odtwarzania dzwiekow dla robota speaker

int close_wnd_speaker_play(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	if (interface.speaker->is_wind_speaker_play_open) {
		PtDestroyWidget(ABW_wnd_speaker_play);
	}

	return (Pt_CONTINUE);

}

int start_wind_speaker_play(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.speaker->is_wind_speaker_play_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_speaker_play, widget, cbinfo);
		interface.speaker->is_wind_speaker_play_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_speaker_play);
	}

	return (Pt_CONTINUE);

}

int clear_wind_speaker_play_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.speaker->is_wind_speaker_play_open = false;
	return (Pt_CONTINUE);

}

// inicjacja okna do odtwarzania dzwiekow dla robota speaker

int init_wnd_speaker_play(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	speaker_check_state(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int speaker_preset_sound_play(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

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

		if (interface.speaker->state.edp.pid != -1) {

			if ((ApName(ApWidget(cbinfo)) == ABN_mm_speaker_preset_sound_0) || ((cbinfo->event->type == Ph_EV_KEY)
					&& (my_data->key_cap == 0x30))) {
				text = interface.speaker->state.edp.preset_sound_0;
				prosody = "neutral";
			} else if ((ApName(ApWidget(cbinfo)) == ABN_mm_speaker_preset_sound_1) || ((cbinfo->event->type
					== Ph_EV_KEY) && (my_data->key_cap == 0x31))) {
				text = interface.speaker->state.edp.preset_sound_1;
				prosody = "neutral";
			} else if ((ApName(ApWidget(cbinfo)) == ABN_mm_speaker_preset_sound_2) || ((cbinfo->event->type
					== Ph_EV_KEY) && (my_data->key_cap == 0x32))) {
				text = interface.speaker->state.edp.preset_sound_2;
				prosody = "neutral";
			}

			interface.speaker->ui_ecp_robot->send_command(text.c_str(), prosody.c_str());

		}

	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

// funkcja wysylajaca rozkaz odtworzenia dzwieku do robota speaker

int speaker_play_exec(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char local_text[lib::MAX_TEXT];
	char local_prosody[lib::MAX_PROSODY];
	char* ref_local_text;
	char* ref_local_prosody;

	// wychwytania ew. bledow ECP::robot
	try {

		if (interface.speaker->state.edp.pid != -1)

			PtGetResource(ABW_PtText_wnd_speaker_play_text_entry,
					Pt_ARG_TEXT_STRING, &ref_local_text, 0);
		PtGetResource(ABW_PtComboBox_wnd_speaker_play_prosody_entry,
				Pt_ARG_TEXT_STRING, &ref_local_prosody, 0);
		strcpy(local_text, ref_local_text);
		strcpy(local_prosody, ref_local_prosody);

		interface.speaker->ui_ecp_robot->send_command(local_text, local_prosody);

		speaker_check_state(widget, apinfo, cbinfo);

	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);
}

int speaker_check_state(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.speaker->state.edp.pid != -1) {
			interface.speaker->ui_ecp_robot->read_state(&(interface.speaker->ui_ecp_robot->speaking_state));

			if (interface.speaker->ui_ecp_robot->speaking_state) { // odtwarzanie w toku
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

int EDP_speaker_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_PROCESS_CREATION);

	//char tmp_string[100];
	//char tmp2_string[100];

	try { // dla bledow robot :: ECP_error

		// dla robota speaker
		if (interface.speaker->state.edp.state == 0) {
			interface.speaker->state.edp.state = 0;
			interface.speaker->state.edp.is_synchronised = false;

			std::string tmp_string("/dev/name/global/");
			tmp_string += interface.speaker->state.edp.hardware_busy_attach_point;

			std::string tmp2_string("/dev/name/global/");
			tmp2_string += interface.speaker->state.edp.network_resourceman_attach_point;

			// sprawdzeie czy nie jest juz zarejestrowany zarzadca zasobow
			if (((!(interface.speaker->state.edp.test_mode)) && (access(tmp_string.c_str(), R_OK) == 0))
					|| (access(tmp2_string.c_str(), R_OK) == 0)) {
				interface.ui_msg->message("edp_speaker already exists");

			} else if (interface.check_node_existence(interface.speaker->state.edp.node_name, "edp_speaker")) {

				interface.speaker->state.edp.node_nr
						= interface.config->return_node_number(interface.speaker->state.edp.node_name);

				interface.speaker->ui_ecp_robot
						= new ui::speaker::EcpRobot(&interface.speaker->state.edp, *interface.config, *interface.all_ecp_msg);
				interface.speaker->state.edp.pid = interface.speaker->ui_ecp_robot->get_EDP_pid();

				if (interface.speaker->state.edp.pid < 0) {
					interface.speaker->state.edp.state = 0;
					fprintf(stderr, "edp spawn failed: %s\n", strerror(errno));
					delete interface.speaker->ui_ecp_robot;
				} else { // jesli spawn sie powiodl

					interface.speaker->state.edp.state = 1;

					//interface.speaker->connect_to_reader();

					//interface.speaker->state.edp.state=1;// edp wlaczone reader czeka na start
					interface.speaker->state.edp.is_synchronised = true;
				}
			}
		}

	} // end try
	CATCH_SECTION_UI

	interface.manage_interface();

	return (Pt_CONTINUE);

}

int EDP_speaker_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.speaker->EDP_slay_int();

	return (Pt_CONTINUE);
}

int pulse_reader_speaker_start(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.speaker->pulse_reader_start_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);
}

int pulse_reader_speaker_stop(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.speaker->pulse_reader_stop_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_reader_speaker_trigger(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.speaker->pulse_reader_trigger_exec_pulse())
		process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_ecp_speaker(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	interface.speaker->pulse_ecp();

	return (Pt_CONTINUE);

}
