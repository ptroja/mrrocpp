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
#include <spawn.h>
#include <process.h>
#include <cassert>

#include "base/lib/sr/srlib.h"
#include "ui/src/ui_class.h"
#include "ui/src/ui_ecp.h"

// #include "ui/src/ui.h"
// Konfigurator (dla PROCESS_SPAWN_RSH)
#include "base/lib/configurator.h"
#include "robot/irp6m/const_irp6m.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6ot_tfg/const_irp6ot_tfg.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6p_tfg/const_irp6p_tfg.h"
#include "robot/sarkofag/const_sarkofag.h"
#include "robot/polycrank/const_polycrank.h"
#include "robot/smb/const_smb.h"
#include "robot/spkm/const_spkm.h"
#include "robot/shead/const_shead.h"
#include "robot/speaker/const_speaker.h"
#include "robot/conveyor/const_conveyor.h"
#include "robot/bird_hand/const_bird_hand.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern ui::common::Interface interface;

// odblokowanie widgetu
int set_ui_busy_state_notification(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_BUSY);

	return (Pt_CONTINUE);
}

int set_ui_ready_state_notification(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_READY);

	return (Pt_CONTINUE);
}

int set_ui_state_notification(UI_NOTIFICATION_STATE_ENUM new_notifacion)
{
	if (new_notifacion != interface.notification_state) {
		int pt_res = PtEnter(0);

		interface.notification_state = new_notifacion;

		switch (new_notifacion)
		{
			case UI_N_STARTING:
				PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "STARTING", 0);
				PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_MAGENTA, 0);
				break;
			case UI_N_READY:
				PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "READY", 0);
				PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_BLUE, 0);
				break;
			case UI_N_BUSY:
				PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "BUSY", 0);
				PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_RED, 0);
				break;
			case UI_N_EXITING:
				PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "EXITING", 0);
				PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_MAGENTA, 0);
				break;
			case UI_N_COMMUNICATION:
				PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "COMMUNICATION", 0);
				PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_RED, 0);
				break;
			case UI_N_SYNCHRONISATION:
				PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "SYNCHRONISATION", 0);
				PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_RED, 0);
				break;
			case UI_N_PROCESS_CREATION:
				PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "PROCESS CREATION", 0);
				PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_RED, 0);
				break;
		}

		PtDamageWidget(ABW_PtLabel_ready_busy);
		PtFlush();

		if (pt_res >= 0)
			PtLeave(0);

		return 1;

	}

	return 0;

}

// zamyka okno proces control

int close_process_control_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.is_process_control_window_open) {
		PtDestroyWidget(ABW_wnd_processes_control);
	}

	return (Pt_CONTINUE);

}

int clear_teaching_window_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.is_teaching_window_open = false;
	return (Pt_CONTINUE);

}

int clear_file_selection_window_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.is_file_selection_window_open = false;
	return (Pt_CONTINUE);

}

int clear_wnd_process_control_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.is_process_control_window_open = false;

	return (Pt_CONTINUE);

}

int start_process_control_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.is_process_control_window_open) {
		ApCreateModule(ABM_wnd_processes_control, ABW_base, NULL);
		interface.is_process_control_window_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_processes_control);
	}
	interface.process_control_window_renew = true;
	return (Pt_CONTINUE);

}

int clear_task_config_window_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	interface.is_task_window_open = false;
	return (Pt_CONTINUE);

}

int start_task_config_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// PtRealizeWidget( ABW_task_config_window );

	if (!interface.is_task_window_open) {
		ApCreateModule(ABM_task_config_window, widget, cbinfo);
		// 	 PtRealizeWidget( ABW_task_config_window );
		task_window_param_actualization(widget, apinfo, cbinfo);
		interface.is_task_window_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_task_config_window);
	}
	return (Pt_CONTINUE);

}

int yes_no_callback(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */

	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(cbinfo)) == ABN_PtButton_yes))
			|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap == 0x79))) // Y
	{
		interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;
	}

	else if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(cbinfo)) == ABN_PtButton_no))
			|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap == 0x6e))) // N

	{
		interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_NO;
	}

	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	PtDestroyWidget(ABW_yes_no_window);

	return (Pt_CONTINUE);

}

int input_integer_callback(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */

	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;
	int* tmp_ptgr;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_input_integer_ok))
			|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap == 0x6f))) // O
	{
		interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;
		PtGetResource(ABW_PtNumericInteger_wind_input_integer_value, Pt_ARG_NUMERIC_VALUE, &(tmp_ptgr), 0);
		interface.ui_ecp_obj->ui_rep.integer_number = *tmp_ptgr;
	} else if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(cbinfo))
			== ABN_PtButton_wind_input_integer_cancel)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
			== 0x63))) // C
	{
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
		interface.ui_ecp_obj->ui_rep.integer_number = 0;
	}
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	PtDestroyWidget(ABW_wnd_input_integer);

	return (Pt_CONTINUE);

}

int input_double_callback(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */

	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;
	double* tmp_ptgr;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_input_double_ok))
			|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap == 0x6f))) // O
	{
		interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;
		PtGetResource(ABW_PtNumericFloat_wind_input_double_value, Pt_ARG_NUMERIC_VALUE, &(tmp_ptgr), 0);
		interface.ui_ecp_obj->ui_rep.double_number = *tmp_ptgr;
	} else if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(cbinfo))
			== ABN_PtButton_wind_input_double_cancel)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
			== 0x63))) // C
	{
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
		interface.ui_ecp_obj->ui_rep.double_number = 0.0;
	}
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	PtDestroyWidget(ABW_wnd_input_double);

	return (Pt_CONTINUE);

}

int choose_option_callback(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */

	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_choose_option_1))
			|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap == 0x31))) // 1
	{
		interface.ui_ecp_obj->ui_rep.reply = lib::OPTION_ONE;
	} else if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(cbinfo))
			== ABN_PtButton_wind_choose_option_2))
			|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap == 0x32))) // 2
	{
		interface.ui_ecp_obj->ui_rep.reply = lib::OPTION_TWO;
	} else if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(cbinfo))
			== ABN_PtButton_wind_choose_option_3))
			|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap == 0x33))) // 3
	{
		interface.ui_ecp_obj->ui_rep.reply = lib::OPTION_THREE;
	} else if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(cbinfo))
			== ABN_PtButton_wind_choose_option_4))
			|| ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap == 0x34))) // 4
	{
		interface.ui_ecp_obj->ui_rep.reply = lib::OPTION_FOUR;
	} else if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(cbinfo))
			== ABN_PtButton_wind_choose_option_cancel)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
			== 0x63))) // C
	{
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	PtDestroyWidget(ABW_wnd_choose_option);

	return (Pt_CONTINUE);

}

int close_file_selection_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */

	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if ((interface.file_window_mode == ui::common::FSTRAJECTORY) && (interface.ui_ecp_obj->communication_state
			!= ui::common::UI_ECP_REPLY_READY)) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	interface.ui_ecp_obj->synchroniser.command();

	PtDestroyWidget(ABW_file_selection_window);

	return (Pt_CONTINUE);

}

int close_teaching_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */

	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	interface.ui_ecp_obj->synchroniser.command();

	PtDestroyWidget(ABW_teaching_window);

	return (Pt_CONTINUE);

}

int close_task_config_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */

	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PtDestroyWidget(ABW_task_config_window);

	return (Pt_CONTINUE);

}

int init_teaching_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// rodzaj polecenia z ECP
	switch (interface.ui_ecp_obj->ecp_to_ui_msg.ecp_message)
	{
		case lib::C_XYZ_ANGLE_AXIS:

			if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6ot_m::ROBOT_NAME) {
				start_wnd_irp6_on_track_xyz_angle_axis(widget, apinfo, cbinfo);
			} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6p_m::ROBOT_NAME) {

				start_wnd_irp6_postument_xyz_angle_axis(widget, apinfo, cbinfo);
			} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6m::ROBOT_NAME) {
				start_wnd_irp6m_xyz_angle_axis(widget, apinfo, cbinfo);
			}
			break;
		case lib::C_XYZ_EULER_ZYZ:

			if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6ot_m::ROBOT_NAME) {
				start_wnd_irp6_on_track_xyz_euler_zyz(widget, apinfo, cbinfo);
			} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6p_m::ROBOT_NAME) {
				start_wnd_irp6_postument_xyz_euler_zyz(widget, apinfo, cbinfo);
			} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6m::ROBOT_NAME) {
				start_wnd_irp6m_xyz_euler_zyz(widget, apinfo, cbinfo);
			}

			break;
		case lib::C_JOINT:
			if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6ot_m::ROBOT_NAME) {
				start_wnd_irp6_on_track_int(widget, apinfo, cbinfo);
			} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6p_m::ROBOT_NAME) {
				start_wnd_irp6_postument_int(widget, apinfo, cbinfo);
			} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6m::ROBOT_NAME) {
				start_wnd_irp6m_int(widget, apinfo, cbinfo);
			}

			break;
		case lib::C_MOTOR:
			if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6ot_m::ROBOT_NAME) {
				start_wnd_irp6_on_track_inc(widget, apinfo, cbinfo);
			} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6p_m::ROBOT_NAME) {
				start_wnd_irp6_postument_inc(widget, apinfo, cbinfo);
			} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6m::ROBOT_NAME) {
				start_wnd_irp6m_inc(widget, apinfo, cbinfo);
			}

			break;
		default:
			break;
	}
	// 		ApCreateModule (ABM_teaching_window, ABW_base, cbinfo);
	// 	PtRealizeWidget( ABW_teaching_window );

	return (Pt_CONTINUE);

}

int teaching_window_end_motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.teachingstate = ui::common::MP_RUNNING;
	interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;

	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;
	PtDestroyWidget(ABW_teaching_window);

	return (Pt_CONTINUE);

}

int file_selection_window_send_location(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	PtFileSelItem_t *item;
	char *buffer;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	item = PtFSGetCurrent(ABW_PtFileSel_sl);

	// dla pliku trajektorii
	if (item != NULL) {
		if (interface.file_window_mode == ui::common::FSTRAJECTORY) {
			if ((item->type) == Pt_FS_FILE) {
				strncpy(interface.ui_ecp_obj->ui_rep.filename, rindex(item->fullpath, '/') + 1, strlen(rindex(item->fullpath, '/'))
						- 1);
				interface.ui_ecp_obj->ui_rep.filename[strlen(rindex(item->fullpath, '/')) - 1] = '\0';
				strncpy(interface.ui_ecp_obj->ui_rep.path, item->fullpath, strlen(item->fullpath)
						- strlen(rindex(item->fullpath, '/')));
				interface.ui_ecp_obj->ui_rep.path[strlen(item->fullpath) - strlen(rindex(item->fullpath, '/'))] = '\0';
			} else if (((item->type) == Pt_FS_DIR_OP) || ((item->type) == Pt_FS_DIR_CL)) {

				strcpy(interface.ui_ecp_obj->ui_rep.path, item->fullpath);
				PtGetResource(ABW_PtText_file_filename, Pt_ARG_TEXT_STRING, &buffer, 0);
				char file_name[strlen(buffer)];
				strcpy(file_name, buffer);
				strcpy(interface.ui_ecp_obj->ui_rep.filename, file_name);
			}

			// kopiowanie biezacej sciezki, aby w nastepnym wywolaniu okna od niej zaczynac
			interface.teach_filesel_fullpath = interface.ui_ecp_obj->ui_rep.path;
			// opuszczenie semaforu dla watku UI_COMM
			interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

			// dla pliku konfiguracyjnego
		} else if (interface.file_window_mode == ui::common::FSCONFIG) {
			if ((item->type) == Pt_FS_FILE) {
				// To sie pozniej sprawdzi, czy wogule jest wzorzec znaleziony
				std::string str_fullpath(item->fullpath);
				std::string str_tail = str_fullpath.substr(str_fullpath.rfind(interface.mrrocpp_local_path)
						+ interface.mrrocpp_local_path.length());
				//fprintf(stderr, "mrrocpp_local_path: %s\n", interface.mrrocpp_local_path.c_str());
				//fprintf(stderr, "fullpath: %s\n", item->fullpath);
				//fprintf(stderr, "tail: %s\n", str_tail.c_str());
				// TODO: what is going on here ?!
				// char buff[PATH_MAX];
				// buff[strlen(rindex(item->fullpath,'/'))-1]='\0';

				// interface.config_file = buff;
				interface.config_file = str_tail;

				PtSetResource(ABW_PtText_config_file, Pt_ARG_TEXT_STRING, interface.config_file.c_str(), 0);
				PtDamageWidget(ABW_PtText_config_file);
			}
		}

		PtDestroyWidget(ABW_file_selection_window);
	}

	return (Pt_CONTINUE);

}

// Strojenie okna wyboru pliku do wyboru pliku trajektorii badz pliku konfiguracyjnego
int file_selection_window_post_realize(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{
	PtFileSelItem_t *item;
	PtFileSelItem_t **item_list;
	PtArg_t args[1];

	char* p;
	char* buffer;
	const char* delims = { "/" };
	char current_path[100]; // biezacy katalog do porwnan

	int wyjscie = 0;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// dla wyboru pliku konfiguracyjnego

	switch (interface.file_window_mode)
	{
		case ui::common::FSCONFIG:
			// 	printf("aaa:\n");
			// ustawienie katalogu root
			PtSetArg(&args[0], Pt_ARG_FS_ROOT_DIR, interface.config_file_fullpath.c_str(), 0);
			PtSetResources(ABW_PtFileSel_sl, 1, args);
			PtDamageWidget(ABW_PtFileSel_sl);

			// zaznaczenie jednego z elementow
			item_list = PtFSAllItems(ABW_PtFileSel_sl, NULL );
			item = *item_list;
			PtFSSelect(ABW_PtFileSel_sl, item);
			break;
		case ui::common::FSTRAJECTORY:
			// printf("bbb:\n");
			// ustawienie katalogu root
			PtSetArg(&args[0], Pt_ARG_FS_ROOT_DIR, "/net", 0);
			PtSetResources(ABW_PtFileSel_sl, 1, args);
			PtDamageWidget(ABW_PtFileSel_sl);

			// przejscie do katalogu z trajektoriami
			buffer = strdup(interface.teach_filesel_fullpath.c_str());
			strcpy(current_path, "");

			// 	    printf( "%s\n", buffer );
			p = strtok(buffer, delims);
			wyjscie = 1;
			while ((p != NULL) && (wyjscie)) {
				wyjscie = 0;
				strcat(current_path, "/");
				strcat(current_path, p);
				// 	     printf( "word: %s\n", current_path );

				item_list = PtFSAllItems(ABW_PtFileSel_sl, NULL );
				for (; !(((*item_list) == NULL) || (wyjscie)); item_list++) {
					item = *item_list;
					if (strcmp(item->fullpath, current_path) == 0)
						wyjscie++;
					// 		printf ("fullpath: %s, opened: %d, type: %d, root: %d, tag: %d\n", item->fullpath,  item->opened, item->type, item->root, item->tag);
				}
				if (wyjscie) {
					PtFSSelect(ABW_PtFileSel_sl, item);
					PtFSFolderExpand(ABW_PtFileSel_sl, item, NULL );
					PtFSGoto(ABW_PtFileSel_sl, item);
					PtFSDamageItem(ABW_PtFileSel_sl, item);
				}
				p = strtok(NULL, delims);
			}
			//   printf( "%s\n", buffer );
			free(buffer);
			break;
		default:

			break;
	}

	return (Pt_CONTINUE);
}

int close_base_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	// if (ApName( widget) == ABN_base) 	printf("UI CLOSED\n");

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	PtDestroyWidget(ABW_base);

	return (Pt_CONTINUE);

}

int quit(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{
	int do_close = 0; // czy zamykac naprawde??

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (cbinfo->reason_subtype == Ph_EV_WM) {

		if (cbinfo->event->type == Ph_EV_WM) {
			// 	printf("aaab\n");
			if (cbinfo->event->subtype == Ph_EV_WM_EVENT) {

				PhWindowEvent_t *my_data;

				my_data = (PhWindowEvent_t *) PhGetData(cbinfo->event);
				if (my_data->event_f == Ph_WM_CLOSE) { // przycisk zamykania okna
					do_close = 1;
				}
			}
		}
	} else { // wywolano to przyciskiem quit z menu
		do_close = 1;
	}

	if (do_close) // jesli apliakcja ma byc zamknieta
	{
		interface.UI_close();
	}
	return (Pt_CONTINUE);

}

// aktualizacja ustawien przyciskow
int process_control_window_init(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.is_process_control_window_open) {

		bool wlacz_PtButton_wnd_processes_control_all_reader_start = false;
		bool wlacz_PtButton_wnd_processes_control_all_reader_stop = false;
		bool wlacz_PtButton_wnd_processes_control_all_reader_trigger = false;

		// Dla READER'A

		interface.block_widget(ABW_PtButton_wnd_processes_control_all_reader_start);
		interface.block_widget(ABW_PtButton_wnd_processes_control_all_reader_stop);
		interface.block_widget(ABW_PtButton_wnd_processes_control_all_reader_trigger);

		// Dla irp6_on_track

		interface.irp6ot_m->process_control_window_irp6ot_section_init(wlacz_PtButton_wnd_processes_control_all_reader_start, wlacz_PtButton_wnd_processes_control_all_reader_stop, wlacz_PtButton_wnd_processes_control_all_reader_trigger);

		// Dla irp6_postument

		interface.irp6p_m->process_control_window_irp6p_section_init(wlacz_PtButton_wnd_processes_control_all_reader_start, wlacz_PtButton_wnd_processes_control_all_reader_stop, wlacz_PtButton_wnd_processes_control_all_reader_trigger);

		// Dla conveyor

		interface.conveyor->process_control_window_conveyor_section_init(wlacz_PtButton_wnd_processes_control_all_reader_start, wlacz_PtButton_wnd_processes_control_all_reader_stop, wlacz_PtButton_wnd_processes_control_all_reader_trigger);

		// Dla speakera - wylaczone

		// Dla irp6_mechatronika

		interface.irp6m_m->process_control_window_irp6m_section_init(wlacz_PtButton_wnd_processes_control_all_reader_start, wlacz_PtButton_wnd_processes_control_all_reader_stop, wlacz_PtButton_wnd_processes_control_all_reader_trigger);

		// All reader's pulse buttons
		if (wlacz_PtButton_wnd_processes_control_all_reader_start) {
			interface.unblock_widget(ABW_PtButton_wnd_processes_control_all_reader_start);
		}

		if (wlacz_PtButton_wnd_processes_control_all_reader_stop) {
			interface.unblock_widget(ABW_PtButton_wnd_processes_control_all_reader_stop);
		}

		if (wlacz_PtButton_wnd_processes_control_all_reader_trigger) {
			interface.unblock_widget(ABW_PtButton_wnd_processes_control_all_reader_trigger);
		}

		// Dla mp i ecp
		if ((interface.mp.state != interface.mp.last_state) || (interface.process_control_window_renew)) {
			interface.process_control_window_renew = false;

			switch (interface.mp.state)
			{
				case ui::common::UI_MP_PERMITED_TO_RUN:
					interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_start);
					interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_stop);
					interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_pause);
					interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_resume);
					interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_trigger);

					block_all_ecp_trigger_widgets(NULL, NULL, NULL);
					break;
				case ui::common::UI_MP_WAITING_FOR_START_PULSE:
					interface.unblock_widget(ABW_PtButton_wnd_processes_control_mp_pulse_start);
					interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_stop);
					interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_pause);
					interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_resume);
					interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_trigger);

					block_all_ecp_trigger_widgets(NULL, NULL, NULL);
					break;
				case ui::common::UI_MP_TASK_RUNNING:
					interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_start);
					interface.unblock_widget(ABW_PtButton_wnd_processes_control_mp_pulse_stop);
					interface.unblock_widget(ABW_PtButton_wnd_processes_control_mp_pulse_pause);
					interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_resume);
					interface.unblock_widget(ABW_PtButton_wnd_processes_control_mp_pulse_trigger);

					unblock_all_ecp_trigger_widgets(NULL, NULL, NULL);
					break;
				case ui::common::UI_MP_TASK_PAUSED:
					interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_start);
					interface.unblock_widget(ABW_PtButton_wnd_processes_control_mp_pulse_stop);
					interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_pause);
					interface.unblock_widget(ABW_PtButton_wnd_processes_control_mp_pulse_resume);
					interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_trigger);

					block_all_ecp_trigger_widgets(NULL, NULL, NULL);
					break;
				default:
					break;
			}

			interface.mp.last_state = interface.mp.state;

		}

		PtDamageWidget(ABW_wnd_processes_control);
	}

	return (Pt_CONTINUE);

}

int block_all_ecp_trigger_widgets(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6ot_m->state.edp.is_synchronised) {
		interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_ecp_trigger);
	}
	if (interface.irp6p_m->state.edp.is_synchronised) {
		interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_ecp_trigger);
	}
	if (interface.conveyor->state.edp.is_synchronised) {
		interface.block_widget(ABW_PtButton_wnd_processes_control_conveyor_ecp_trigger);
	}
	if (interface.speaker->state.edp.is_synchronised) {
		interface.block_widget(ABW_PtButton_wnd_processes_control_speaker_ecp_trigger);
	}
	if (interface.irp6m_m->state.edp.is_synchronised) {
		interface.block_widget(ABW_PtButton_wnd_processes_control_irp6m_ecp_trigger);
	}
	interface.block_widget(ABW_PtButton_wnd_processes_control_all_ecp_trigger);

	return (Pt_CONTINUE);
}

int unblock_all_ecp_trigger_widgets(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.irp6ot_m->state.edp.is_synchronised) {
		interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_ecp_trigger);
	}
	if (interface.irp6p_m->state.edp.is_synchronised) {
		interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6p_ecp_trigger);
	}
	if (interface.conveyor->state.edp.is_synchronised) {
		interface.unblock_widget(ABW_PtButton_wnd_processes_control_conveyor_ecp_trigger);
	}
	if (interface.speaker->state.edp.is_synchronised) {
		interface.unblock_widget(ABW_PtButton_wnd_processes_control_speaker_ecp_trigger);
	}
	if (interface.irp6m_m->state.edp.is_synchronised) {
		interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6m_ecp_trigger);
	}
	interface.unblock_widget(ABW_PtButton_wnd_processes_control_all_ecp_trigger);

	return (Pt_CONTINUE);
}

int task_param_actualization(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char* tmp_buf;

	PtGetResource(ABW_PtText_config_file, Pt_ARG_TEXT_STRING, &tmp_buf, 0);
	interface.config_file = tmp_buf;

	return (Pt_CONTINUE);
}

int task_window_param_actualization(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	/*			printf("aaa: %s\n",interface.mp_name);
	 printf("bbb: %s\n",interface.mp_node_name);*/

	PtSetResource(ABW_PtText_config_file, Pt_ARG_TEXT_STRING, interface.config_file.c_str(), 0);
	PtSetResource(ABW_PtLabel_bin_directory, Pt_ARG_TEXT_STRING, interface.binaries_network_path.c_str(), 0);

	return (Pt_CONTINUE);
}

int manage_configuration_file(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	task_param_actualization(widget, apinfo, cbinfo);
	interface.reload_whole_configuration();

	interface.set_default_configuration_file_name(); // zapis do pliku domyslnej konfiguracji
	// sprawdza czy sa postawione gns's i ew. stawia je
	// uwaga serwer musi byc wczesniej postawiony
	interface.check_gns();
	task_window_param_actualization(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int start_file_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.is_file_selection_window_open) {
		interface.is_file_selection_window_open = 1;
		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_browse_config_file) {
			interface.file_window_mode = ui::common::FSCONFIG; // wybor pliku konfiguracyjnego
		} else {
			interface.file_window_mode = ui::common::FSTRAJECTORY; // wybor pliku z trajektoria
		}
		ApCreateModule(ABM_file_selection_window, ABW_base, NULL);
	} else {
		// 	printf("Okno file selection jest juz otwarte\n");
	}
	return (Pt_CONTINUE);
}

int clear_console(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	int cur_line = 1;
	int line_width = -2;

	int attributes_mask = 0;
	PtMultiTextAttributes_t attr;

	// czyszczenie kolejnych linii
	do {
		PtMultiTextInfo(ABW_PtMultiText_sr_window, Pt_MT_QUERY_LINE, NULL, &cur_line, NULL, NULL, NULL, &line_width, NULL, NULL);
		if (line_width != 0)
			PtMultiTextModifyText(ABW_PtMultiText_sr_window, 0, line_width, NULL, NULL, NULL, &attr, attributes_mask);
	} while (line_width != 0);
	PtDamageWidget(ABW_PtMultiText_sr_window);

	return (Pt_CONTINUE);

}

// zatrzymuje zadanie, zabija procesy
int unload_all(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	MPslay(widget, apinfo, cbinfo);
	delay(200);
	EDP_all_robots_slay(widget, apinfo, cbinfo);
	close_process_control_window(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

// najpierw unload_all zabija wszystkie procesy wzmiankowane w pliku konfiguracyjnym

int slay_all(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// program unload

	unload_all(widget, apinfo, cbinfo);

	// brutal overkilling

	for (std::list <ui::common::program_node_def>::iterator program_node_list_iterator =
			interface.program_node_list.begin(); program_node_list_iterator != interface.program_node_list.end(); program_node_list_iterator++) {
		char system_command[100];
		/*
		 #if 0 && defined(PROCESS_SPAWN_RSH)
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

#if 0 && defined(PROCESS_SPAWN_RSH)
		sprintf(system_command, "rsh %s killall -e -q -v %s",
				program_node_list_iterator->node_name.c_str(),
				program_node_list_iterator->program_name.c_str()
		);
#else
		sprintf(system_command, "slay -9 -v -f -n %s %s", program_node_list_iterator->node_name.c_str(), program_node_list_iterator->program_name.c_str());
#endif
		printf("bbb: %s\n", system_command);
		system(system_command);
	}

	interface.manage_interface();

	return (Pt_CONTINUE);

}

int activate_menu_from_widget(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	struct
	{
		PhEvent_t event;
		PhRect_t rect;
		PhPointerEvent_t pevent;
	} new_event;

	memset(&new_event.rect, -1, sizeof(new_event.rect));

	if (cbinfo->event) {
		new_event.event = *(cbinfo->event);
	}

	new_event.event.processing_flags = Ph_FAKE_EVENT;
	new_event.event.type = Ph_EV_BUT_PRESS;
	new_event.event.subtype = Ph_EV_RELEASE_PHANTOM;
	new_event.pevent.click_count = 1;
	new_event.pevent.buttons = Ph_BUTTON_SELECT;
	new_event.event.num_rects = 1;
	PtSendEventToWidget(widget, (PhEvent_t *) &new_event);

	return (Pt_CONTINUE);

}

int activate_file_menu(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// printf("activate_file_menu\n");
	activate_menu_from_widget(ABW_base_file, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int activate_robot_menu(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	activate_menu_from_widget(ABW_base_robot, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int activate_all_robots_menu(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// printf("activate_file_menu\n");
	activate_menu_from_widget(ABW_base_all_robots, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int activate_task_menu(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// printf("activate_file_menu\n");
	activate_menu_from_widget(ABW_base_task, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int activate_special_menu(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// printf("activate_file_menu\n");
	activate_menu_from_widget(ABW_base_special, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int activate_help_menu(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// printf("activate_file_menu\n");
	activate_menu_from_widget(ABW_base_help, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int close_yes_no_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	interface.ui_ecp_obj->synchroniser.command();
	return (Pt_CONTINUE);

}

int close_input_integer_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	interface.ui_ecp_obj->synchroniser.command();
	return (Pt_CONTINUE);

}

int close_input_double_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	interface.ui_ecp_obj->synchroniser.command();
	return (Pt_CONTINUE);

}

int close_choose_option_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	interface.ui_ecp_obj->synchroniser.command();
	return (Pt_CONTINUE);

}

int EDP_all_robots_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	EDP_conveyor_synchronise(widget, apinfo, cbinfo);
	EDP_irp6_on_track_synchronise(widget, apinfo, cbinfo);
	EDP_irp6ot_tfg_synchronise(widget, apinfo, cbinfo);
	EDP_irp6p_tfg_synchronise(widget, apinfo, cbinfo);
	EDP_sarkofag_synchronise(widget, apinfo, cbinfo);
	EDP_irp6_postument_synchronise(widget, apinfo, cbinfo);
	EDP_irp6_mechatronika_synchronise(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int teaching_window_send_move(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	double *motion_time;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PtGetResource(ABW_PtNumericFloat_move_time, Pt_ARG_NUMERIC_VALUE, &motion_time, 0);

	if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6ot_m::ROBOT_NAME) {
		for (int i = 0; i < lib::irp6ot_m::NUM_OF_SERVOS; i++)
			interface.ui_ecp_obj->ui_rep.coordinates[i] = interface.irp6ot_m->irp6ot_current_pos[i];
	} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6p_m::ROBOT_NAME) {
		for (int i = 0; i < lib::irp6p_m::NUM_OF_SERVOS; i++)
			interface.ui_ecp_obj->ui_rep.coordinates[i] = interface.irp6p_m->irp6p_current_pos[i];
	} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6m::ROBOT_NAME) {
		for (int i = 0; i < lib::irp6m::NUM_OF_SERVOS; i++)
			interface.ui_ecp_obj->ui_rep.coordinates[i] = interface.irp6m_m->irp6m_current_pos[i];
	}

	interface.ui_ecp_obj->ui_rep.double_number = *motion_time;
	interface.ui_ecp_obj->ui_rep.reply = lib::NEXT;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;
	interface.ui_ecp_obj->synchroniser.command();

	return (Pt_CONTINUE);
}

int all_robots_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((interface.mp.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN) || (interface.mp.state
			== ui::common::UI_MP_PERMITED_TO_RUN) || (interface.mp.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {
		// ruch do pozcyji synchronizacji dla Irp6_on_track i dla dalszych analogicznie
		if (interface.check_synchronised_and_loaded(interface.irp6ot_m->state))
			irp6ot_move_to_preset_position(widget, apinfo, cbinfo);
		if (interface.check_synchronised_and_loaded(interface.irp6ot_tfg->state))
			irp6ot_tfg_move_to_preset_position(widget, apinfo, cbinfo);
		if (interface.check_synchronised_and_loaded(interface.irp6p_m->state))
			irp6p_move_to_preset_position(widget, apinfo, cbinfo);
		if (interface.check_synchronised_and_loaded(interface.irp6p_tfg->state))
			irp6p_tfg_move_to_preset_position(widget, apinfo, cbinfo);
		if (interface.check_synchronised_and_loaded(interface.sarkofag->state))
			sarkofag_move_to_preset_position(widget, apinfo, cbinfo);
		if (interface.check_synchronised_and_loaded(interface.conveyor->state))
			conveyor_move_to_preset_position(widget, apinfo, cbinfo);
		if (interface.check_synchronised_and_loaded(interface.irp6m_m->state))
			irp6m_move_to_preset_position(widget, apinfo, cbinfo);
	}

	return (Pt_CONTINUE);

}

int EDP_all_robots_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	EDP_irp6_on_track_create(widget, apinfo, cbinfo);
	EDP_irp6ot_tfg_create(widget, apinfo, cbinfo);
	EDP_irp6_postument_create(widget, apinfo, cbinfo);
	EDP_irp6p_tfg_create(widget, apinfo, cbinfo);
	EDP_sarkofag_create(widget, apinfo, cbinfo);
	EDP_conveyor_create(widget, apinfo, cbinfo);
	EDP_bird_hand_create(widget, apinfo, cbinfo);
	EDP_spkm_create(widget, apinfo, cbinfo);
	EDP_smb_create(widget, apinfo, cbinfo);
	EDP_shead_create(widget, apinfo, cbinfo);
	EDP_speaker_create(widget, apinfo, cbinfo);
	EDP_irp6_mechatronika_create(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int EDP_all_robots_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	EDP_irp6_on_track_slay(widget, apinfo, cbinfo);
	EDP_irp6ot_tfg_slay(widget, apinfo, cbinfo);
	EDP_irp6_postument_slay(widget, apinfo, cbinfo);
	EDP_irp6p_tfg_slay(widget, apinfo, cbinfo);
	EDP_sarkofag_slay(widget, apinfo, cbinfo);
	EDP_conveyor_slay(widget, apinfo, cbinfo);
	EDP_bird_hand_slay(widget, apinfo, cbinfo);
	EDP_spkm_slay(widget, apinfo, cbinfo);
	EDP_smb_slay(widget, apinfo, cbinfo);
	EDP_shead_slay(widget, apinfo, cbinfo);
	EDP_speaker_slay(widget, apinfo, cbinfo);
	EDP_irp6_mechatronika_slay(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int MPup(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.main_eb.command(boost::bind(MPup_int, widget, apinfo, cbinfo));

	return (Pt_CONTINUE);

}

int MPup_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	int pt_res;
	set_ui_state_notification(UI_N_PROCESS_CREATION);

	if (interface.mp.pid == -1) {

		interface.mp.node_nr = interface.config->return_node_number(interface.mp.node_name.c_str());

		std::string mp_network_pulse_attach_point("/dev/name/global/");
		mp_network_pulse_attach_point += interface.mp.network_pulse_attach_point;

		// sprawdzenie czy nie jest juz zarejestrowany serwer komunikacyjny MP
		if (access(mp_network_pulse_attach_point.c_str(), R_OK) == 0) {
			interface.ui_msg->message(lib::NON_FATAL_ERROR, "mp already exists");
		} else if (interface.check_node_existence(interface.mp.node_name, "mp")) {
			interface.mp.pid = interface.config->process_spawn(lib::MP_SECTION);

			if (interface.mp.pid > 0) {

				short tmp = 0;
				// kilka sekund  (~1) na otworzenie urzadzenia
				while ((interface.mp.pulse_fd
						= name_open(interface.mp.network_pulse_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) < 0)
					if ((tmp++) < lib::CONNECT_RETRY)
						delay(lib::CONNECT_DELAY);
					else {
						fprintf(stderr, "name_open() for %s failed: %s\n", interface.mp.network_pulse_attach_point.c_str(), strerror(errno));
						break;
					}

				interface.teachingstate = ui::common::MP_RUNNING;

				interface.mp.state = ui::common::UI_MP_WAITING_FOR_START_PULSE; // mp wlaczone
				pt_res = PtEnter(0);
				start_process_control_window(widget, apinfo, cbinfo);
				if (pt_res >= 0)
					PtLeave(0);
			} else {
				fprintf(stderr, "mp spawn failed\n");
			}
			interface.manage_interface();
		}
	}

	return 1;
}

int MPslay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.mp.pid != -1) {

		if ((interface.mp.state == ui::common::UI_MP_TASK_RUNNING) || (interface.mp.state
				== ui::common::UI_MP_TASK_PAUSED)) {

			pulse_stop_mp(widget, apinfo, cbinfo);
		}

		name_close(interface.mp.pulse_fd);

		// 	printf("dddd: %d\n", SignalKill(ini_con->mp-
		// 	printf("mp slay\n");
		SignalKill(interface.mp.node_nr, interface.mp.pid, 0, SIGTERM, 0, 0);
		interface.mp.state = ui::common::UI_MP_PERMITED_TO_RUN; // mp wylaczone

	}
	// delay(1000);
	// 	kill(interface.mp_pid,SIGTERM);
	// 	printf("mp pupa po kill\n");
	interface.mp.pid = -1;
	interface.mp.pulse_fd = -1;

	interface.deactivate_ecp_trigger(interface.irp6ot_m->state);
	interface.deactivate_ecp_trigger(interface.irp6p_m->state);
	interface.deactivate_ecp_trigger(interface.conveyor->state);
	interface.deactivate_ecp_trigger(interface.speaker->state);
	interface.deactivate_ecp_trigger(interface.irp6m_m->state);

	// modyfikacja menu
	interface.manage_interface();
	process_control_window_init(widget, apinfo, cbinfo);
	return (Pt_CONTINUE);

}

int pulse_start_mp(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.mp.state == ui::common::UI_MP_WAITING_FOR_START_PULSE) {

		interface.mp.state = ui::common::UI_MP_TASK_RUNNING;// czekanie na stop

		// zamkniecie okien ruchow recznych o ile sa otwarte

		interface.irp6ot_m->close_all_windows();
		interface.irp6p_m->close_all_windows();
		interface.irp6m_m->close_all_windows();
		interface.bird_hand->close_all_windows();
		interface.conveyor->close_all_windows();
		interface.irp6ot_tfg->close_all_windows();
		interface.irp6p_tfg->close_all_windows();
		interface.sarkofag->close_all_windows();
		interface.speaker->close_all_windows();

		interface.execute_mp_pulse(MP_START);

		process_control_window_init(widget, apinfo, cbinfo);

		interface.manage_interface();
	}

	return (Pt_CONTINUE);

}

int pulse_stop_mp(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if ((interface.mp.state == ui::common::UI_MP_TASK_RUNNING) || (interface.mp.state == ui::common::UI_MP_TASK_PAUSED)) {

		interface.mp.state = ui::common::UI_MP_WAITING_FOR_START_PULSE;// czekanie na stop

		interface.execute_mp_pulse(MP_STOP);

		process_control_window_init(widget, apinfo, cbinfo);

		interface.manage_interface();
	}

	return (Pt_CONTINUE);

}

int pulse_pause_mp(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.mp.state == ui::common::UI_MP_TASK_RUNNING) {

		interface.mp.state = ui::common::UI_MP_TASK_PAUSED;// czekanie na stop

		interface.execute_mp_pulse(MP_PAUSE);

		process_control_window_init(widget, apinfo, cbinfo);

		interface.manage_interface();
	}

	return (Pt_CONTINUE);

}

int pulse_resume_mp(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.mp.state == ui::common::UI_MP_TASK_PAUSED) {

		interface.mp.state = ui::common::UI_MP_TASK_RUNNING;// czekanie na stop

		interface.execute_mp_pulse(MP_RESUME);

		process_control_window_init(widget, apinfo, cbinfo);

		interface.manage_interface();
	}

	return (Pt_CONTINUE);

}

int pulse_trigger_mp(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.mp.state == ui::common::UI_MP_TASK_RUNNING) {

		interface.execute_mp_pulse(MP_TRIGGER);

		process_control_window_init(widget, apinfo, cbinfo);

		interface.manage_interface();
	}

	return (Pt_CONTINUE);

}

int signal_mp(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	fprintf(stderr, "UI: %s:%d signal_mp() -- thish should not happend!\n", __FILE__, __LINE__);
	assert(0);
#if 0
	/*
	 * "To jest kod ktory jest b. stary i praktycznie nie uzywany.
	 *  Mozesz w calosci wyremowac wnetrze funkcji zostawiajac
	 *  jednakze sama funkcje ze wzgledu na callbacki."
	 *  (T.Winiarski, 28/05/2007)
	 */
	int ret, signo;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wnd_processes_control_signal_start) {
		signo=SIGUSR1;// by Y - tymczasowo
	} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wnd_processes_control_signal_stop) {
		signo=SIGUSR1;// by Y - tymczasowo
	} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wnd_processes_control_signal_pause) {
		signo=SIGSTOP;
		interface.teachingstate = MP_PAUSED_H;
		// Zawieszenie procesow MP, ECP i EDP sygnalem SIGSTOP
	} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wnd_processes_control_signal_resume) {
		// Odwieszenie procesow EDP, ECP i MP sygnalem
		signo=SIGCONT;
		interface.teachingstate = ui::common::MP_RUNNING;
	}

	/*int SignalKill( uint32_t nd,
	 pid_t pid,
	 int tid,
	 int signo,
	 int code,
	 int value );
	 */

	// if((	signo!=SIGCONT)&&(signo!=SIGSTOP)) {

	if ( ( ret = SignalKill(interface.irp6ot_m->state.edp.node_nr, interface.irp6ot_m->state.edp.pid, 1, signo,0,0) ) == -1 ) { // by Y !!! klopoty z wysylaniem do okreslonego watku -
		// wstawiona maska na odbior sygnalow po stronie serwo
		// 	perror("UI: Stop EDP failed");
		printf("sending a signal to edp failed\n");

	}
	// 	 }
	// XXX probably a bug - killing _ecp_ pid on _edp_ node (ptrojane)
	if ( ( ret = SignalKill(interface.irp6ot_m->state.edp.node_nr, interface.irp6ot_m->state.ecp.pid, 0, signo,0,0) ) == -1 ) {
		// 	perror("UI: Stop ECP failed");
		printf("sending a signal to ecp failed\n");

	}
	if ( ( ret = SignalKill(interface.mp.node_nr, interface.mp.pid, 0, signo,0,0) ) == -1 ) {
		// 	perror("UI: Stop MP failed");
		printf("sending a signal to mp failed\n");

	}

#endif
	return (Pt_CONTINUE);
}

int pulse_reader_all_robots_start(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6ot_m->pulse_reader_start_exec_pulse();
	interface.irp6p_m->pulse_reader_start_exec_pulse();
	interface.conveyor->pulse_reader_start_exec_pulse();
	interface.irp6m_m->pulse_reader_start_exec_pulse();

	process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_reader_all_robots_stop(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6ot_m->pulse_reader_stop_exec_pulse();
	interface.irp6p_m->pulse_reader_stop_exec_pulse();
	interface.conveyor->pulse_reader_stop_exec_pulse();
	interface.irp6m_m->pulse_reader_stop_exec_pulse();
	process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_reader_all_robots_trigger(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.irp6ot_m->pulse_reader_trigger_exec_pulse();
	interface.irp6p_m->pulse_reader_trigger_exec_pulse();
	interface.conveyor->pulse_reader_trigger_exec_pulse();
	interface.irp6m_m->pulse_reader_trigger_exec_pulse();

	return (Pt_CONTINUE);

}

int pulse_ecp_all_robots(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	pulse_ecp_irp6_on_track(widget, apinfo, cbinfo);
	pulse_ecp_irp6_postument(widget, apinfo, cbinfo);
	pulse_ecp_conveyor(widget, apinfo, cbinfo);
	pulse_ecp_speaker(widget, apinfo, cbinfo);
	pulse_ecp_irp6_mechatronika(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

