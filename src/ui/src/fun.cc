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
#include <spawn.h>
#include <process.h>
#include <assert.h>

#include "lib/srlib.h"
#include "ui/ui_const.h"
#include "ui/ui_class.h"

// #include "ui/ui.h"
// Konfigurator (dla PROCESS_SPAWN_RSH)
#include "lib/configurator.h"
#include "ui/ui_ecp.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern Ui ui;

extern function_execution_buffer main_eb;

extern ui_ecp_buffer* ui_ecp_obj;

extern ui_state_def ui_state;

ui_robot_def ui_robot;
extern ui_ecp_buffer* ui_ecp_obj;

extern double irp6ot_current_pos[8]; // pozycja biezaca
extern double irp6ot_desired_pos[8]; // pozycja zadana

extern double irp6p_current_pos[7]; // pozycja biezaca
extern double irp6p_desired_pos[7]; // pozycja zadana

extern double irp6m_current_pos[6]; // pozycja biezaca
extern double irp6m_desired_pos[6]; // pozycja zadana


// blokowanie widgetu
int block_widget(PtWidget_t *widget) {
	PtSetResource(widget, Pt_ARG_FLAGS, Pt_TRUE, Pt_BLOCKED | Pt_GHOST);
	PtDamageWidget(widget);

	return 1;
}

// odblokowanie widgetu
int unblock_widget(PtWidget_t *widget) {
	PtSetResource(widget, Pt_ARG_FLAGS, Pt_FALSE, Pt_BLOCKED | Pt_GHOST);
	PtDamageWidget(widget);

	return 1;
}

// odblokowanie widgetu
int set_ui_busy_state_notification(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_BUSY);

	return (Pt_CONTINUE);
}

int set_ui_ready_state_notification(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_READY);

	return (Pt_CONTINUE);
}

int set_ui_state_notification(UI_NOTIFICATION_STATE_ENUM new_notifacion) {
	if (new_notifacion != ui.notification_state) {
		int pt_res = PtEnter(0);

		ui.notification_state = new_notifacion;

		switch (new_notifacion) {
		case UI_N_STARTING:
			PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING,
					"STARTING", 0);
			PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_MAGENTA, 0);
			break;
		case UI_N_READY:
			PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "READY",
					0);
			PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_BLUE, 0);
			break;
		case UI_N_BUSY:
			PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING, "BUSY", 0);
			PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_RED, 0);
			break;
		case UI_N_EXITING:
			PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING,
					"EXITING", 0);
			PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_MAGENTA, 0);
			break;
		case UI_N_COMMUNICATION:
			PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING,
					"COMMUNICATION", 0);
			PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_RED, 0);
			break;
		case UI_N_SYNCHRONISATION:
			PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING,
					"SYNCHRONISATION", 0);
			PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_COLOR, Pg_RED, 0);
			break;
		case UI_N_PROCESS_CREATION:
			PtSetResource(ABW_PtLabel_ready_busy, Pt_ARG_TEXT_STRING,
					"PROCESS CREATION", 0);
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

int close_process_control_window(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.is_process_control_window_open) {
		PtDestroyWidget(ABW_wnd_processes_control);
	}

	return (Pt_CONTINUE);

}

int clear_teaching_window_flag(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui.is_teaching_window_open = false;
	return (Pt_CONTINUE);

}

int clear_file_selection_window_flag(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui.is_file_selection_window_open = false;
	return (Pt_CONTINUE);

}

int clear_wnd_process_control_flag(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui.is_process_control_window_open = false;

	return (Pt_CONTINUE);

}

int start_process_control_window(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!ui.is_process_control_window_open) {
		ApCreateModule(ABM_wnd_processes_control, ABW_base, NULL);
		ui.is_process_control_window_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_processes_control);
	}
	ui.process_control_window_renew = true;
	return (Pt_CONTINUE);

}

int clear_task_config_window_flag(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	ui.is_task_window_open = false;
	return (Pt_CONTINUE);

}

int start_task_config_window(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// PtRealizeWidget( ABW_task_config_window );

	if (!ui.is_task_window_open) {
		ApCreateModule(ABM_task_config_window, widget, cbinfo);
		// 	 PtRealizeWidget( ABW_task_config_window );
		task_window_param_actualization(widget, apinfo, cbinfo);
		ui.is_task_window_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_task_config_window);
	}
	return (Pt_CONTINUE);

}

int yes_no_callback(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {

	/* eliminate 'unreferenced' warnings */

	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	if (((cbinfo->event->type == Ph_EV_BUT_RELEASE)
			&& (ApName(ApWidget(cbinfo)) == ABN_PtButton_yes))
			|| ((cbinfo->event->type == Ph_EV_KEY)
					&& (my_data->key_cap == 0x79))) // Y
	{
		ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;
	}

	else if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(
			cbinfo)) == ABN_PtButton_no))
			|| ((cbinfo->event->type == Ph_EV_KEY)
					&& (my_data->key_cap == 0x6e))) // N

	{
		ui_ecp_obj->ui_rep.reply = lib::ANSWER_NO;
	}

	ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;

	PtDestroyWidget(ABW_yes_no_window);

	return (Pt_CONTINUE);

}

int input_integer_callback(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */

	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;
	int* tmp_ptgr;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	if (((cbinfo->event->type == Ph_EV_BUT_RELEASE)
			&& (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_input_integer_ok))
			|| ((cbinfo->event->type == Ph_EV_KEY)
					&& (my_data->key_cap == 0x6f))) // O
	{
		ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;
		PtGetResource(ABW_PtNumericInteger_wind_input_integer_value,
				Pt_ARG_NUMERIC_VALUE, &(tmp_ptgr), 0);
		ui_ecp_obj->ui_rep.integer_number = *tmp_ptgr;
	} else if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(
			cbinfo)) == ABN_PtButton_wind_input_integer_cancel))
			|| ((cbinfo->event->type == Ph_EV_KEY)
					&& (my_data->key_cap == 0x63))) // C
	{
		ui_ecp_obj->ui_rep.reply = lib::QUIT;
		ui_ecp_obj->ui_rep.integer_number = 0;
	}
	ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;

	PtDestroyWidget(ABW_wnd_input_integer);

	return (Pt_CONTINUE);

}

int input_double_callback(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */

	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;
	double* tmp_ptgr;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	if (((cbinfo->event->type == Ph_EV_BUT_RELEASE)
			&& (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_input_double_ok))
			|| ((cbinfo->event->type == Ph_EV_KEY)
					&& (my_data->key_cap == 0x6f))) // O
	{
		ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;
		PtGetResource(ABW_PtNumericFloat_wind_input_double_value,
				Pt_ARG_NUMERIC_VALUE, &(tmp_ptgr), 0);
		ui_ecp_obj->ui_rep.double_number = *tmp_ptgr;
	} else if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(
			cbinfo)) == ABN_PtButton_wind_input_double_cancel))
			|| ((cbinfo->event->type == Ph_EV_KEY)
					&& (my_data->key_cap == 0x63))) // C
	{
		ui_ecp_obj->ui_rep.reply = lib::QUIT;
		ui_ecp_obj->ui_rep.double_number = 0.0;
	}
	ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;

	PtDestroyWidget(ABW_wnd_input_double);

	return (Pt_CONTINUE);

}

int choose_option_callback(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */

	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	if (((cbinfo->event->type == Ph_EV_BUT_RELEASE)
			&& (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_choose_option_1))
			|| ((cbinfo->event->type == Ph_EV_KEY)
					&& (my_data->key_cap == 0x31))) // 1
	{
		ui_ecp_obj->ui_rep.reply = lib::OPTION_ONE;
	} else if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(
			cbinfo)) == ABN_PtButton_wind_choose_option_2))
			|| ((cbinfo->event->type == Ph_EV_KEY)
					&& (my_data->key_cap == 0x32))) // 2
	{
		ui_ecp_obj->ui_rep.reply = lib::OPTION_TWO;
	} else if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(
			cbinfo)) == ABN_PtButton_wind_choose_option_3))
			|| ((cbinfo->event->type == Ph_EV_KEY)
					&& (my_data->key_cap == 0x33))) // 3
	{
		ui_ecp_obj->ui_rep.reply = lib::OPTION_THREE;
	} else if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(
			cbinfo)) == ABN_PtButton_wind_choose_option_4))
			|| ((cbinfo->event->type == Ph_EV_KEY)
					&& (my_data->key_cap == 0x34))) // 4
	{
		ui_ecp_obj->ui_rep.reply = lib::OPTION_FOUR;
	} else if (((cbinfo->event->type == Ph_EV_BUT_RELEASE) && (ApName(ApWidget(
			cbinfo)) == ABN_PtButton_wind_choose_option_cancel))
			|| ((cbinfo->event->type == Ph_EV_KEY)
					&& (my_data->key_cap == 0x63))) // C
	{
		ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;

	PtDestroyWidget(ABW_wnd_choose_option);

	return (Pt_CONTINUE);

}

int close_file_selection_window(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */

	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if ((ui.file_window_mode == FSTRAJECTORY)
			&& (ui_ecp_obj->communication_state != UI_ECP_REPLY_READY)) {
		ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	ui_ecp_obj->synchroniser.command();

	PtDestroyWidget(ABW_file_selection_window);

	return (Pt_CONTINUE);

}

int close_teaching_window(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */

	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_ecp_obj->communication_state != UI_ECP_REPLY_READY) {
		ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	ui_ecp_obj->synchroniser.command();

	PtDestroyWidget(ABW_teaching_window);

	return (Pt_CONTINUE);

}

int close_task_config_window(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */

	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PtDestroyWidget(ABW_task_config_window);

	return (Pt_CONTINUE);

}

int init_teaching_window(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// rodzaj polecenia z ECP
	switch (ui_ecp_obj->ecp_to_ui_msg.ecp_message) {
	case lib::C_XYZ_ANGLE_AXIS:
		switch (ui_ecp_obj->ecp_to_ui_msg.robot_name) {
		case lib::ROBOT_IRP6OT_M:
			start_wnd_irp6_on_track_xyz_angle_axis(widget, apinfo, cbinfo);
			break;
		case lib::ROBOT_IRP6P_M:
			start_wnd_irp6_postument_xyz_angle_axis(widget, apinfo, cbinfo);
			break;
		case lib::ROBOT_IRP6_MECHATRONIKA:
			start_wnd_irp6m_xyz_angle_axis(widget, apinfo, cbinfo);
			break;
		default:
			break;
		}
		break;
	case lib::C_XYZ_EULER_ZYZ:
		switch (ui_ecp_obj->ecp_to_ui_msg.robot_name) {
		case lib::ROBOT_IRP6OT_M:
			start_wnd_irp6_on_track_xyz_euler_zyz(widget, apinfo, cbinfo);
			break;
		case lib::ROBOT_IRP6P_M:
			start_wnd_irp6_postument_xyz_euler_zyz(widget, apinfo, cbinfo);
			break;
		case lib::ROBOT_IRP6_MECHATRONIKA:
			start_wnd_irp6m_xyz_euler_zyz(widget, apinfo, cbinfo);
			break;
		default:
			break;
		}
		break;
	case lib::C_JOINT:
		switch (ui_ecp_obj->ecp_to_ui_msg.robot_name) {
		case lib::ROBOT_IRP6OT_M:
			start_wnd_irp6_on_track_int(widget, apinfo, cbinfo);
			break;
		case lib::ROBOT_IRP6P_M:
			start_wnd_irp6_postument_int(widget, apinfo, cbinfo);
			break;
		case lib::ROBOT_IRP6_MECHATRONIKA:
			start_wnd_irp6m_int(widget, apinfo, cbinfo);
			break;
		default:
			break;
		}
		break;
	case lib::C_MOTOR:
		switch (ui_ecp_obj->ecp_to_ui_msg.robot_name) {
		case lib::ROBOT_IRP6OT_M:
			start_wnd_irp6_on_track_inc(widget, apinfo, cbinfo);
			break;
		case lib::ROBOT_IRP6P_M:
			start_wnd_irp6_postument_inc(widget, apinfo, cbinfo);
			break;
		case lib::ROBOT_IRP6_MECHATRONIKA:
			start_wnd_irp6m_inc(widget, apinfo, cbinfo);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	// 		ApCreateModule (ABM_teaching_window, ABW_base, cbinfo);
	// 	PtRealizeWidget( ABW_teaching_window );

	return (Pt_CONTINUE);

}

int teaching_window_end_motion(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui.teachingstate = MP_RUNNING;
	ui_ecp_obj->ui_rep.reply = lib::QUIT;

	ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;
	PtDestroyWidget(ABW_teaching_window);

	return (Pt_CONTINUE);

}

int file_selection_window_send_location(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	PtFileSelItem_t *item;
	char *buffer;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	item = PtFSGetCurrent(ABW_PtFileSel_sl);

	// dla pliku trajektorii
	if (item != NULL) {
		if (ui.file_window_mode == FSTRAJECTORY) {
			if ((item->type) == Pt_FS_FILE) {
				strncpy(ui_ecp_obj->ui_rep.filename,
						rindex(item->fullpath, '/') + 1, strlen(rindex(
								item->fullpath, '/')) - 1);
				ui_ecp_obj->ui_rep.filename[strlen(rindex(item->fullpath, '/'))
						- 1] = '\0';
				strncpy(ui_ecp_obj->ui_rep.path, item->fullpath, strlen(
						item->fullpath) - strlen(rindex(item->fullpath, '/')));
				ui_ecp_obj->ui_rep.path[strlen(item->fullpath) - strlen(rindex(
						item->fullpath, '/'))] = '\0';
			} else if (((item->type) == Pt_FS_DIR_OP) || ((item->type)
					== Pt_FS_DIR_CL)) {

				strcpy(ui_ecp_obj->ui_rep.path, item->fullpath);
				PtGetResource(ABW_PtText_file_filename, Pt_ARG_TEXT_STRING,
						&buffer, 0);
				char file_name[strlen(buffer)];
				strcpy(file_name, buffer);
				strcpy(ui_ecp_obj->ui_rep.filename, file_name);
			}

			// kopiowanie biezacej sciezki, aby w nastepnym wywolaniu okna od niej zaczynac
			ui.teach_filesel_fullpath = ui_ecp_obj->ui_rep.path;
			// opuszczenie semaforu dla watku UI_COMM
			ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;

			// dla pliku konfiguracyjnego
		} else if (ui.file_window_mode == FSCONFIG) {
			if ((item->type) == Pt_FS_FILE) {
				// To sie pozniej sprawdzi, czy wogule jest wzorzec znaleziony
				std::string str_fullpath(item->fullpath);
				std::string str_tail =
						str_fullpath.substr(str_fullpath.rfind(
								ui.mrrocpp_local_path)
								+ ui.mrrocpp_local_path.length());
				//fprintf(stderr, "mrrocpp_local_path: %s\n", ui.mrrocpp_local_path.c_str());
				//fprintf(stderr, "fullpath: %s\n", item->fullpath);
				//fprintf(stderr, "tail: %s\n", str_tail.c_str());
				// TODO: what is going on here ?!
				// char buff[PATH_MAX];
				// buff[strlen(rindex(item->fullpath,'/'))-1]='\0';

				// ui.config_file = buff;
				ui.config_file = str_tail;

				PtSetResource(ABW_PtText_config_file, Pt_ARG_TEXT_STRING,
						ui.config_file.c_str(), 0);
				PtDamageWidget(ABW_PtText_config_file);
			}
		}

		PtDestroyWidget(ABW_file_selection_window);
	}

	return (Pt_CONTINUE);

}

// Strojenie okna wyboru pliku do wyboru pliku trajektorii badz pliku konfiguracyjnego
int file_selection_window_post_realize(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

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

	switch (ui.file_window_mode) {
	case FSCONFIG:
		// 	printf("aaa:\n");
		// ustawienie katalogu root
		PtSetArg(&args[0], Pt_ARG_FS_ROOT_DIR, ui.config_file_fullpath.c_str(),
				0);
		PtSetResources(ABW_PtFileSel_sl, 1, args);
		PtDamageWidget(ABW_PtFileSel_sl);

		// zaznaczenie jednego z elementow
		item_list = PtFSAllItems(ABW_PtFileSel_sl, NULL );
		item = *item_list;
		PtFSSelect(ABW_PtFileSel_sl, item);
		break;
	case FSTRAJECTORY:
		// printf("bbb:\n");
		// ustawienie katalogu root
		PtSetArg(&args[0], Pt_ARG_FS_ROOT_DIR, "/net", 0);
		PtSetResources(ABW_PtFileSel_sl, 1, args);
		PtDamageWidget(ABW_PtFileSel_sl);

		// przejscie do katalogu z trajektoriami
		buffer = strdup(ui.teach_filesel_fullpath.c_str());
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

int close_base_window(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

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
		ui.UI_close();
	}
	return (Pt_CONTINUE);

}

// aktualizacja ustawien przyciskow
int process_control_window_init(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.is_process_control_window_open) {

		bool wlacz_PtButton_wnd_processes_control_all_reader_start = false;
		bool wlacz_PtButton_wnd_processes_control_all_reader_stop = false;
		bool wlacz_PtButton_wnd_processes_control_all_reader_trigger = false;

		// Dla READER'A

		block_widget(ABW_PtButton_wnd_processes_control_all_reader_start);
		block_widget(ABW_PtButton_wnd_processes_control_all_reader_stop);
		block_widget(ABW_PtButton_wnd_processes_control_all_reader_trigger);

		// Dla irp6_on_track

		process_control_window_irp6ot_section_init(
				wlacz_PtButton_wnd_processes_control_all_reader_start,
				wlacz_PtButton_wnd_processes_control_all_reader_stop,
				wlacz_PtButton_wnd_processes_control_all_reader_trigger);

		// Dla irp6_postument

		process_control_window_irp6p_section_init(
				wlacz_PtButton_wnd_processes_control_all_reader_start,
				wlacz_PtButton_wnd_processes_control_all_reader_stop,
				wlacz_PtButton_wnd_processes_control_all_reader_trigger);

		// Dla conveyor

		process_control_window_conveyor_section_init(
				wlacz_PtButton_wnd_processes_control_all_reader_start,
				wlacz_PtButton_wnd_processes_control_all_reader_stop,
				wlacz_PtButton_wnd_processes_control_all_reader_trigger);

		// Dla speakera - wylaczone

		// Dla irp6_mechatronika

		process_control_window_irp6m_section_init(
				wlacz_PtButton_wnd_processes_control_all_reader_start,
				wlacz_PtButton_wnd_processes_control_all_reader_stop,
				wlacz_PtButton_wnd_processes_control_all_reader_trigger);

		// All reader's pulse buttons
		if (wlacz_PtButton_wnd_processes_control_all_reader_start) {
			unblock_widget(ABW_PtButton_wnd_processes_control_all_reader_start);
		}

		if (wlacz_PtButton_wnd_processes_control_all_reader_stop) {
			unblock_widget(ABW_PtButton_wnd_processes_control_all_reader_stop);
		}

		if (wlacz_PtButton_wnd_processes_control_all_reader_trigger) {
			unblock_widget(
					ABW_PtButton_wnd_processes_control_all_reader_trigger);
		}

		// Dla mp i ecp
		if ((ui.mp.state != ui.mp.last_state)
				|| (ui.process_control_window_renew)) {
			ui.process_control_window_renew = false;

			switch (ui.mp.state) {
			case UI_MP_PERMITED_TO_RUN:
				block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_start);
				block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_stop);
				block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_pause);
				block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_resume);
				block_widget(
						ABW_PtButton_wnd_processes_control_mp_pulse_trigger);

				block_all_ecp_trigger_widgets(NULL, NULL, NULL);
				break;
			case UI_MP_WAITING_FOR_START_PULSE:
				unblock_widget(
						ABW_PtButton_wnd_processes_control_mp_pulse_start);
				block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_stop);
				block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_pause);
				block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_resume);
				block_widget(
						ABW_PtButton_wnd_processes_control_mp_pulse_trigger);

				block_all_ecp_trigger_widgets(NULL, NULL, NULL);
				break;
			case UI_MP_TASK_RUNNING:
				block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_start);
				unblock_widget(ABW_PtButton_wnd_processes_control_mp_pulse_stop);
				unblock_widget(
						ABW_PtButton_wnd_processes_control_mp_pulse_pause);
				block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_resume);
				unblock_widget(
						ABW_PtButton_wnd_processes_control_mp_pulse_trigger);

				unblock_all_ecp_trigger_widgets(NULL, NULL, NULL);
				break;
			case UI_MP_TASK_PAUSED:
				block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_start);
				unblock_widget(ABW_PtButton_wnd_processes_control_mp_pulse_stop);
				block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_pause);
				unblock_widget(
						ABW_PtButton_wnd_processes_control_mp_pulse_resume);
				block_widget(
						ABW_PtButton_wnd_processes_control_mp_pulse_trigger);

				block_all_ecp_trigger_widgets(NULL, NULL, NULL);
				break;
			default:
				break;
			}

			ui.mp.last_state = ui.mp.state;

		}

		PtDamageWidget(ABW_wnd_processes_control);
	}

	return (Pt_CONTINUE);

}

int block_all_ecp_trigger_widgets(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.irp6ot_m.state.edp.is_synchronised) {
		block_widget(ABW_PtButton_wnd_processes_control_irp6ot_ecp_trigger);
	}
	if (ui_state.irp6p_m.edp.is_synchronised) {
		block_widget(ABW_PtButton_wnd_processes_control_irp6p_ecp_trigger);
	}
	if (ui_state.conveyor.edp.is_synchronised) {
		block_widget(ABW_PtButton_wnd_processes_control_conveyor_ecp_trigger);
	}
	if (ui_state.speaker.edp.is_synchronised) {
		block_widget(ABW_PtButton_wnd_processes_control_speaker_ecp_trigger);
	}
	if (ui_state.irp6_mechatronika.edp.is_synchronised) {
		block_widget(ABW_PtButton_wnd_processes_control_irp6m_ecp_trigger);
	}
	block_widget(ABW_PtButton_wnd_processes_control_all_ecp_trigger);

	return (Pt_CONTINUE);
}

int unblock_all_ecp_trigger_widgets(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.irp6ot_m.state.edp.is_synchronised) {
		unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_ecp_trigger);
	}
	if (ui_state.irp6p_m.edp.is_synchronised) {
		unblock_widget(ABW_PtButton_wnd_processes_control_irp6p_ecp_trigger);
	}
	if (ui_state.conveyor.edp.is_synchronised) {
		unblock_widget(ABW_PtButton_wnd_processes_control_conveyor_ecp_trigger);
	}
	if (ui_state.speaker.edp.is_synchronised) {
		unblock_widget(ABW_PtButton_wnd_processes_control_speaker_ecp_trigger);
	}
	if (ui_state.irp6_mechatronika.edp.is_synchronised) {
		unblock_widget(ABW_PtButton_wnd_processes_control_irp6m_ecp_trigger);
	}
	unblock_widget(ABW_PtButton_wnd_processes_control_all_ecp_trigger);

	return (Pt_CONTINUE);
}

int task_param_actualization(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	char* tmp_buf;

	PtGetResource(ABW_PtText_config_file, Pt_ARG_TEXT_STRING, &tmp_buf, 0);
	ui.config_file = tmp_buf;

	return (Pt_CONTINUE);
}

int task_window_param_actualization(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	/*			printf("aaa: %s\n",ui.mp_name);
	 printf("bbb: %s\n",ui.mp_node_name);*/

	PtSetResource(ABW_PtText_config_file, Pt_ARG_TEXT_STRING,
			ui.config_file.c_str(), 0);
	PtSetResource(ABW_PtLabel_bin_directory, Pt_ARG_TEXT_STRING,
			ui.binaries_network_path.c_str(), 0);

	return (Pt_CONTINUE);
}

int clear_all_configuration_lists() {
	// clearing of lists
	ui.section_list.clear();
	ui.config_node_list.clear();
	ui.all_node_list.clear();
	ui.program_node_list.clear();

	return 1;
}

int initiate_configuration() {
	if (access(ui.config_file_relativepath.c_str(), R_OK) != 0) {
		fprintf(
				stderr,
				"Wrong entry in default_file.cfg - load another configuration than: %s\n",
				ui.config_file_relativepath.c_str());
		ui.config_file_relativepath = "../configs/common.ini";
	}

	// sprawdzenie czy nazwa sesji jest unikalna

	bool wyjscie = false;

	while (!wyjscie) {
		time_t now = time(NULL);
		char now_string[32];
		strftime(now_string, 8, "_%H%M%S", localtime(&now));
		ui.session_name = now_string;

		if (ui.config) {
			delete ui.config;
		}
		ui.config = new lib::configurator(ui.ui_node_name,
				ui.mrrocpp_local_path, ui.config_file, UI_SECTION,
				ui.session_name);

		std::string attach_point =
				ui.config->return_attach_point_name(
						lib::configurator::CONFIG_SERVER, "sr_attach_point",
						UI_SECTION);

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

	ui.ui_attach_point = ui.config->return_attach_point_name(
			lib::configurator::CONFIG_SERVER, "ui_attach_point", UI_SECTION);
	ui.sr_attach_point = ui.config->return_attach_point_name(
			lib::configurator::CONFIG_SERVER, "sr_attach_point", UI_SECTION);
	ui.network_sr_attach_point = ui.config->return_attach_point_name(
			lib::configurator::CONFIG_SERVER, "sr_attach_point", UI_SECTION);

	clear_all_configuration_lists();

	// sczytanie listy sekcji
	fill_section_list(ui.config_file_relativepath.c_str());
	fill_section_list("../configs/common.ini");
	fill_node_list();
	fill_program_node_list();

	return 1;
}

// fills section list of configuration files
int fill_section_list(const char* file_name_and_path) {
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

			std::list<Ui::list_t>::iterator list_iterator;

			// checking if section is already considered
			for (list_iterator = ui.section_list.begin(); list_iterator
					!= ui.section_list.end(); list_iterator++) {
				if ((*list_iterator) == current_section)
					break;
			}

			// if the section does not exists
			if (list_iterator == ui.section_list.end()) {
				ui.section_list.push_back(std::string(current_section));
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
int fill_node_list() {
	// fill all network nodes list

	DIR* dirp = opendir("/net");
	if (dirp != NULL) {
		for (;;) {
			struct dirent *direntp = readdir(dirp);
			if (direntp == NULL)
				break;
			ui.all_node_list.push_back(std::string(direntp->d_name));
		}
		closedir(dirp);
	}

	for (std::list<Ui::list_t>::iterator section_list_iterator =
			ui.section_list.begin(); section_list_iterator
			!= ui.section_list.end(); section_list_iterator++) {
		if (ui.config->exists("node_name", *section_list_iterator)) {
			std::string tmp = ui.config->value<std::string> ("node_name",
					*section_list_iterator);

			std::list<Ui::list_t>::iterator node_list_iterator;

			for (node_list_iterator = ui.config_node_list.begin(); node_list_iterator
					!= ui.config_node_list.end(); node_list_iterator++) {
				if (tmp == (*node_list_iterator)) {
					break;
				}
			}

			// if the node does not exists
			if (node_list_iterator == ui.config_node_list.end()) {
				ui.config_node_list.push_back(tmp);
			}
		}

	}

	return 1;
}

// fills program_node list
int fill_program_node_list() {
	//	printf("fill_program_node_list\n");

	for (std::list<Ui::list_t>::iterator section_list_iterator =
			ui.section_list.begin(); section_list_iterator
			!= ui.section_list.end(); section_list_iterator++) {

		if ((ui.config->exists("program_name", *section_list_iterator)
				&& ui.config->exists("node_name", *section_list_iterator))) {
			//	char* tmp_p =ui.config->value<std::string>("program_name", *section_list_iterator);
			//	char* tmp_n =ui.config->value<std::string>("node_name", *section_list_iterator);

			program_node_def tmp_s;

			tmp_s.program_name = ui.config->value<std::string> ("program_name",
					*section_list_iterator);
			tmp_s.node_name = ui.config->value<std::string> ("node_name",
					*section_list_iterator);

			ui.program_node_list.push_back(tmp_s);
		}
	}

	return 1;
}

int manage_configuration_file(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	task_param_actualization(widget, apinfo, cbinfo);
	ui.reload_whole_configuration();

	set_default_configuration_file_name(); // zapis do pliku domyslnej konfiguracji
	// sprawdza czy sa postawione gns's i ew. stawia je
	// uwaga serwer musi byc wczesniej postawiony
	check_gns();
	task_window_param_actualization(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

// odczytuje nazwe domyslengo pliku konfiguracyjnego, w razie braku ustawia common.ini
int get_default_configuration_file_name() {

	FILE * fp = fopen("../configs/default_file.cfg", "r");
	if (fp != NULL) {
		//printf("alala\n");
		char tmp_buf[255];
		fgets(tmp_buf, 255, fp); // Uwaga na zwracanego NULLa
		char *tmp_buf1 = strtok(tmp_buf, "=\n\r"); // get first token
		ui.config_file = tmp_buf1;

		ui.config_file_relativepath = "../";
		ui.config_file_relativepath += ui.config_file;

		fclose(fp);
		return 1;

	} else {
		//	printf("balala\n");
		// jesli plik z domyslna konfiguracja (default_file.cfg) nie istnieje to utworz go i wpisz do niego common.ini
		printf("Utworzono plik default_file.cfg z konfiguracja common.ini\n");
		fp = fopen("../configs/default_file.cfg", "w");
		fclose(fp);

		ui.config_file = "configs/common.ini";
		ui.config_file_relativepath = "../";
		ui.config_file_relativepath += ui.config_file;

		std::ofstream outfile("../configs/default_file.cfg", std::ios::out);
		if (!outfile.good()) {
			std::cerr << "Cannot open file: default_file.cfg" << std::endl;
			perror("because of");
		} else
			outfile << ui.config_file;

		return 2;
	}
}

// zapisuje nazwe domyslengo pliku konfiguracyjnego
int set_default_configuration_file_name() {

	ui.config_file_relativepath = "../";
	ui.config_file_relativepath += ui.config_file;

	std::ofstream outfile("../configs/default_file.cfg", std::ios::out);
	if (!outfile.good()) {
		std::cerr << "Cannot open file: default_file.cfg\n";
		perror("because of");
	} else
		outfile << ui.config_file;

	return 1;
}

int start_file_window(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!ui.is_file_selection_window_open) {
		ui.is_file_selection_window_open = 1;
		if (ApName(ApWidget(cbinfo)) == ABN_PtButton_browse_config_file) {
			ui.file_window_mode = FSCONFIG; // wybor pliku konfiguracyjnego
		} else {
			ui.file_window_mode = FSTRAJECTORY; // wybor pliku z trajektoria
		}
		ApCreateModule(ABM_file_selection_window, ABW_base, NULL);
	} else {
		// 	printf("Okno file selection jest juz otwarte\n");
	}
	return (Pt_CONTINUE);
}

// ustala stan wszytkich EDP
bool check_synchronised_or_inactive(ecp_edp_ui_robot_def& robot) {
	return (((robot.is_active) && (robot.edp.is_synchronised))
			|| (!(robot.is_active)));

}

bool check_synchronised_and_loaded(ecp_edp_ui_robot_def& robot) {
	return (((robot.edp.state > 0) && (robot.edp.is_synchronised)));

}

bool check_loaded_or_inactive(ecp_edp_ui_robot_def& robot) {
	return (((robot.is_active) && (robot.edp.state > 0))
			|| (!(robot.is_active)));

}

bool check_loaded(ecp_edp_ui_robot_def& robot) {
	return ((robot.is_active) && (robot.edp.state > 0));
}

// ustala stan wszytkich EDP
int check_edps_state_and_modify_mp_state() {

	// wyznaczenie stanu wszytkich EDP abstahujac od MP

	// jesli wszytkie sa nieaktywne
	if ((!(ui_state.irp6p_m.is_active)) && (!(ui.irp6ot_m.state.is_active))
			&& (!(ui.irp6ot_tfg.state.is_active))
			&& (!(ui_state.irp6p_tfg.is_active))
			&& (!(ui_state.conveyor.is_active))
			&& (!(ui_state.speaker.is_active))
			&& (!(ui_state.irp6_mechatronika.is_active))
			&& (!(ui.bird_hand.state.is_active))
			&& (!(ui_state.spkm.is_active)) && (!(ui_state.smb.is_active))
			&& (!(ui_state.shead.is_active))) {
		ui.all_edps = UI_ALL_EDPS_NONE_EDP_ACTIVATED;

		// jesli wszystkie sa zsynchronizowane
	} else if (check_synchronised_or_inactive(ui_state.irp6p_m)
			&& check_synchronised_or_inactive(ui.irp6ot_m.state)
			&& check_synchronised_or_inactive(ui_state.conveyor)
			&& check_synchronised_or_inactive(ui_state.speaker)
			&& check_synchronised_or_inactive(ui_state.irp6_mechatronika)
			&& check_synchronised_or_inactive(ui.irp6ot_tfg.state)
			&& check_synchronised_or_inactive(ui_state.irp6p_tfg)
			&& check_synchronised_or_inactive(ui.bird_hand.state)
			&& check_synchronised_or_inactive(ui_state.spkm)
			&& check_synchronised_or_inactive(ui_state.smb)
			&& check_synchronised_or_inactive(ui_state.shead)) {
		ui.all_edps = UI_ALL_EDPS_LOADED_AND_SYNCHRONISED;

		// jesli wszystkie sa zaladowane
	} else if (check_loaded_or_inactive(ui_state.irp6p_m)
			&& check_loaded_or_inactive(ui.irp6ot_m.state)
			&& check_loaded_or_inactive(ui_state.conveyor)
			&& check_loaded_or_inactive(ui_state.speaker)
			&& check_loaded_or_inactive(ui_state.irp6_mechatronika)
			&& check_loaded_or_inactive(ui.irp6ot_tfg.state)
			&& check_loaded_or_inactive(ui_state.irp6p_tfg)
			&& check_loaded_or_inactive(ui.bird_hand.state)
			&& check_loaded_or_inactive(ui_state.spkm)
			&& check_loaded_or_inactive(ui_state.smb)
			&& check_loaded_or_inactive(ui_state.shead))

	{
		ui.all_edps = UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED;

		// jesli chociaz jeden jest zaladowany
	} else if (check_loaded(ui_state.irp6p_m)
			|| check_loaded(ui.irp6ot_m.state) || check_loaded(
			ui_state.conveyor) || check_loaded(ui_state.speaker)
			|| check_loaded(ui_state.irp6_mechatronika) || check_loaded(
			ui.irp6ot_tfg.state) || check_loaded(ui_state.irp6p_tfg)
			|| check_loaded(ui.bird_hand.state) || check_loaded(ui_state.spkm)
			|| check_loaded(ui_state.smb) || check_loaded(ui_state.shead))

	{
		ui.all_edps = UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED;

		// jesli zaden nie jest zaladowany
	} else {
		ui.all_edps = UI_ALL_EDPS_NONE_EDP_LOADED;

	}

	// modyfikacja stanu MP przez stan wysztkich EDP

	switch (ui.all_edps) {
	case UI_ALL_EDPS_NONE_EDP_ACTIVATED:
	case UI_ALL_EDPS_LOADED_AND_SYNCHRONISED:
		if ((ui.mp.state == UI_MP_NOT_PERMITED_TO_RUN)
				&& (ui.is_mp_and_ecps_active)) {
			ui.mp.state = UI_MP_PERMITED_TO_RUN; // pozwol na uruchomienie mp
		}
		break;

	case UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED:
	case UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED:
	case UI_ALL_EDPS_NONE_EDP_LOADED:
		if (ui.mp.state == UI_MP_PERMITED_TO_RUN) {
			ui.mp.state = UI_MP_NOT_PERMITED_TO_RUN; // nie pozwol na uruchomienie mp
		}
		break;
	default:
		break;
	}
	return 1;
}

int clear_console(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	int cur_line = 1;
	int line_width = -2;

	int attributes_mask = 0;
	PtMultiTextAttributes_t attr;

	// czyszczenie kolejnych linii
	do {
		PtMultiTextInfo(ABW_PtMultiText_sr_window, Pt_MT_QUERY_LINE, NULL,
				&cur_line, NULL, NULL, NULL, &line_width, NULL, NULL);
		if (line_width != 0)
			PtMultiTextModifyText(ABW_PtMultiText_sr_window, 0, line_width,
					NULL, NULL, NULL, &attr, attributes_mask);
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

int slay_all(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo) {

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// program unload

	unload_all(widget, apinfo, cbinfo);

	// brutal overkilling

	for (std::list<program_node_def>::iterator program_node_list_iterator =
			ui.program_node_list.begin(); program_node_list_iterator
			!= ui.program_node_list.end(); program_node_list_iterator++) {
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
		sprintf(system_command, "slay -9 -v -f -n %s %s",
				program_node_list_iterator->node_name.c_str(),
				program_node_list_iterator->program_name.c_str());
#endif
		printf("bbb: %s\n", system_command);
		system(system_command);
	}

	ui.manage_interface();

	return (Pt_CONTINUE);

}

// sprawdza czy sa postawione gns's i ew. stawia je
// uwaga serwer powinien byc wczesniej postawiony (dokladnie jeden w sieci)

int check_gns() {
	if (access("/etc/system/config/useqnet", R_OK)) {
		printf(
				"UI: There is no /etc/system/config/useqnet file; the qnet will not work properly.\n");
		PtExit(EXIT_SUCCESS);
	}

	unsigned short number_of_gns_servers = 0;
	std::string gns_server_node;

	// poszukiwanie serwerow gns
	for (std::list<Ui::list_t>::iterator node_list_iterator =
			ui.all_node_list.begin(); node_list_iterator
			!= ui.all_node_list.end(); node_list_iterator++) {
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
		printf(
				"UI: There is more than one gns server in the QNX network; the qnet will not work properly.\n");
		// printing of gns server nodes
		for (std::list<Ui::list_t>::iterator node_list_iterator =
				ui.all_node_list.begin(); node_list_iterator
				!= ui.all_node_list.end(); node_list_iterator++) {
			std::string opendir_path("/net/");

			opendir_path += *node_list_iterator;
			opendir_path += "/proc/mount/dev/name/gns_server";

			// sprawdzenie czy dziala serwer gns
			if (access(opendir_path.c_str(), R_OK) == 0) {
				printf("There is gns server on %s node\n",
						(*node_list_iterator).c_str());
			}
		}
		PtExit(EXIT_SUCCESS);
	}
	// gns server was not found in the QNX network
	else if (!number_of_gns_servers) {
		printf(
				"UI: gns server was not found in the QNX network, it will be automatically run on local node\n");

		// ew. zabicie klienta gns

		if (access("/dev/name", R_OK) == 0) {
			system("slay gns");
		}

		// uruchomienie serwera
		system("gns -s");

		// poszukiwanie serwerow gns
		for (std::list<Ui::list_t>::iterator node_list_iterator =
				ui.all_node_list.begin(); node_list_iterator
				!= ui.all_node_list.end(); node_list_iterator++) {
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
	for (std::list<Ui::list_t>::iterator node_list_iterator =
			ui.config_node_list.begin(); node_list_iterator
			!= ui.config_node_list.end(); node_list_iterator++) {
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

				system(system_command.c_str());
			}

		} else {
			fprintf(
					stderr,
					"check_gns - Nie wykryto wezla: %s, ktory wystepuje w pliku konfiguracyjnym\n",
					(*node_list_iterator).c_str());

			if ((ui.is_sr_thread_loaded) && (ui.ui_msg != NULL)) {
				std::string tmp;
				tmp = std::string("check_gns - Nie wykryto wezla: ")
						+ (*node_list_iterator) + std::string(
						", ktory wystepuje w pliku konfiguracyjnym");
				ui.ui_msg->message(lib::NON_FATAL_ERROR, tmp);
			}

		}
	}

	return (Pt_CONTINUE);

}

int activate_menu_from_widget(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	struct {
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

int activate_file_menu(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// printf("activate_file_menu\n");
	activate_menu_from_widget(ABW_base_file, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int activate_robot_menu(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	activate_menu_from_widget(ABW_base_robot, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int activate_all_robots_menu(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// printf("activate_file_menu\n");
	activate_menu_from_widget(ABW_base_all_robots, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int activate_task_menu(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// printf("activate_file_menu\n");
	activate_menu_from_widget(ABW_base_task, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int activate_special_menu(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// printf("activate_file_menu\n");
	activate_menu_from_widget(ABW_base_special, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int activate_help_menu(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// printf("activate_file_menu\n");
	activate_menu_from_widget(ABW_base_help, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int close_yes_no_window(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_ecp_obj->communication_state != UI_ECP_REPLY_READY) {
		ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	ui_ecp_obj->synchroniser.command();
	return (Pt_CONTINUE);

}

int close_input_integer_window(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_ecp_obj->communication_state != UI_ECP_REPLY_READY) {
		ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	ui_ecp_obj->synchroniser.command();
	return (Pt_CONTINUE);

}

int close_input_double_window(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_ecp_obj->communication_state != UI_ECP_REPLY_READY) {
		ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	ui_ecp_obj->synchroniser.command();
	return (Pt_CONTINUE);

}

int close_choose_option_window(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_ecp_obj->communication_state != UI_ECP_REPLY_READY) {
		ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	ui_ecp_obj->synchroniser.command();
	return (Pt_CONTINUE);

}

int EDP_all_robots_synchronise(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	EDP_conveyor_synchronise(widget, apinfo, cbinfo);
	EDP_irp6_on_track_synchronise(widget, apinfo, cbinfo);
	EDP_irp6ot_tfg_synchronise(widget, apinfo, cbinfo);
	EDP_irp6p_tfg_synchronise(widget, apinfo, cbinfo);
	EDP_irp6_postument_synchronise(widget, apinfo, cbinfo);
	EDP_irp6_mechatronika_synchronise(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int teaching_window_send_move(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {

	double *motion_time;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PtGetResource(ABW_PtNumericFloat_move_time, Pt_ARG_NUMERIC_VALUE,
			&motion_time, 0);

	switch (ui_ecp_obj->ecp_to_ui_msg.robot_name) {
	case lib::ROBOT_IRP6OT_M:
		for (int i = 0; i < IRP6OT_M_NUM_OF_SERVOS; i++)
			ui_ecp_obj->ui_rep.coordinates[i] = irp6ot_current_pos[i];
		break;
	case lib::ROBOT_IRP6P_M:
		for (int i = 0; i < IRP6P_M_NUM_OF_SERVOS; i++)
			ui_ecp_obj->ui_rep.coordinates[i] = irp6p_current_pos[i];
		break;
	case lib::ROBOT_IRP6_MECHATRONIKA:
		for (int i = 0; i < IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++)
			ui_ecp_obj->ui_rep.coordinates[i] = irp6m_current_pos[i];
		break;
	default:
		break;
	}

	ui_ecp_obj->ui_rep.double_number = *motion_time;
	ui_ecp_obj->ui_rep.reply = lib::NEXT;
	ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;
	ui_ecp_obj->synchroniser.command();

	return (Pt_CONTINUE);
}

int all_robots_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((ui.mp.state == UI_MP_NOT_PERMITED_TO_RUN) || (ui.mp.state
			== UI_MP_PERMITED_TO_RUN) || (ui.mp.state
			== UI_MP_WAITING_FOR_START_PULSE)) {
		// ruch do pozcyji synchronizacji dla Irp6_on_track i dla dalszych analogicznie
		if (check_synchronised_and_loaded(ui.irp6ot_m.state))
			irp6ot_move_to_preset_position(widget, apinfo, cbinfo);
		if (check_synchronised_and_loaded(ui.irp6ot_tfg.state))
			irp6ot_tfg_move_to_preset_position(widget, apinfo, cbinfo);
		if (check_synchronised_and_loaded(ui_state.irp6p_m))
			irp6p_move_to_preset_position(widget, apinfo, cbinfo);
		if (check_synchronised_and_loaded(ui_state.irp6p_tfg))
			irp6p_tfg_move_to_preset_position(widget, apinfo, cbinfo);
		if (check_synchronised_and_loaded(ui_state.conveyor))
			conveyor_move_to_preset_position(widget, apinfo, cbinfo);
		if (check_synchronised_and_loaded(ui_state.irp6_mechatronika))
			irp6m_move_to_preset_position(widget, apinfo, cbinfo);
	}

	return (Pt_CONTINUE);

}

int EDP_all_robots_create(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	EDP_irp6_on_track_create(widget, apinfo, cbinfo);
	EDP_irp6ot_tfg_create(widget, apinfo, cbinfo);
	EDP_irp6_postument_create(widget, apinfo, cbinfo);
	EDP_irp6p_tfg_create(widget, apinfo, cbinfo);
	EDP_conveyor_create(widget, apinfo, cbinfo);
	EDP_bird_hand_create(widget, apinfo, cbinfo);
	EDP_spkm_create(widget, apinfo, cbinfo);
	EDP_smb_create(widget, apinfo, cbinfo);
	EDP_shead_create(widget, apinfo, cbinfo);
	EDP_speaker_create(widget, apinfo, cbinfo);
	EDP_irp6_mechatronika_create(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int EDP_all_robots_slay(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	EDP_irp6_on_track_slay(widget, apinfo, cbinfo);
	EDP_irp6ot_tfg_slay(widget, apinfo, cbinfo);
	EDP_irp6_postument_slay(widget, apinfo, cbinfo);
	EDP_irp6p_tfg_slay(widget, apinfo, cbinfo);
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

	//	EDP_irp6_on_track_create_int(widget, apinfo, cbinfo);


	main_eb.command(boost::bind(MPup_int, widget, apinfo, cbinfo));

	return (Pt_CONTINUE);

}

int MPup_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	int pt_res;
	set_ui_state_notification(UI_N_PROCESS_CREATION);

	if (ui.mp.pid == -1) {

		ui.mp.node_nr = ui.config->return_node_number(ui.mp.node_name.c_str());

		std::string mp_network_pulse_attach_point("/dev/name/global/");
		mp_network_pulse_attach_point += ui.mp.network_pulse_attach_point;

		// sprawdzenie czy nie jest juz zarejestrowany serwer komunikacyjny MP
		if (access(mp_network_pulse_attach_point.c_str(), R_OK) == 0) {
			ui.ui_msg->message(lib::NON_FATAL_ERROR, "MP already exists");
		} else if (check_node_existence(ui.mp.node_name, std::string("mp"))) {
			ui.mp.pid = ui.config->process_spawn(MP_SECTION);

			if (ui.mp.pid > 0) {

				short tmp = 0;
				// kilka sekund  (~1) na otworzenie urzadzenia
				while ((ui.mp.pulse_fd = name_open(
						ui.mp.network_pulse_attach_point.c_str(),
						NAME_FLAG_ATTACH_GLOBAL)) < 0)
					if ((tmp++) < CONNECT_RETRY)
						delay(CONNECT_DELAY);
					else {
						fprintf(stderr, "name_open() for %s failed: %s\n",
								ui.mp.network_pulse_attach_point.c_str(),
								strerror(errno));
						break;
					}

				ui.teachingstate = MP_RUNNING;

				ui.mp.state = UI_MP_WAITING_FOR_START_PULSE; // mp wlaczone
				pt_res = PtEnter(0);
				start_process_control_window(widget, apinfo, cbinfo);
				if (pt_res >= 0)
					PtLeave(0);
			} else {
				fprintf(stderr, "MP spawn failed\n");
			}
			ui.manage_interface();
		}
	}

	return 1;
}

int MPslay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.mp.pid != -1) {

		if ((ui.mp.state == UI_MP_TASK_RUNNING) || (ui.mp.state
				== UI_MP_TASK_PAUSED)) {

			pulse_stop_mp(widget, apinfo, cbinfo);
		}

		name_close(ui.mp.pulse_fd);

		// 	printf("dddd: %d\n", SignalKill(ini_con->mp-
		// 	printf("MP slay\n");
		SignalKill(ui.mp.node_nr, ui.mp.pid, 0, SIGTERM, 0, 0);
		ui.mp.state = UI_MP_PERMITED_TO_RUN; // mp wylaczone

	}
	// delay(1000);
	// 	kill(ui.mp_pid,SIGTERM);
	// 	printf("MP pupa po kill\n");
	ui.mp.pid = -1;
	ui.mp.pulse_fd = -1;

	deactivate_ecp_trigger(ui.irp6ot_m.state);
	deactivate_ecp_trigger(ui_state.irp6p_m);
	deactivate_ecp_trigger(ui_state.conveyor);
	deactivate_ecp_trigger(ui_state.speaker);
	deactivate_ecp_trigger(ui_state.irp6_mechatronika);

	// modyfikacja menu
	ui.manage_interface();
	process_control_window_init(widget, apinfo, cbinfo);
	return (Pt_CONTINUE);

}

bool deactivate_ecp_trigger(ecp_edp_ui_robot_def& robot_l) {

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

int pulse_start_mp(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.mp.state == UI_MP_WAITING_FOR_START_PULSE) {

		ui.mp.state = UI_MP_TASK_RUNNING;// czekanie na stop

		// zamkniecie okien ruchow recznych o ile sa otwarte

		close_all_irp6ot_windows(NULL, NULL, NULL);

		close_all_irp6p_windows(NULL, NULL, NULL);

		close_all_irp6m_windows(NULL, NULL, NULL);

		close_wind_conveyor_moves(NULL, NULL, NULL);
		close_wnd_conveyor_servo_algorithm(NULL, NULL, NULL);

		close_wind_irp6ot_tfg_moves(NULL, NULL, NULL);
		close_wnd_irp6ot_tfg_servo_algorithm(NULL, NULL, NULL);

		close_wind_irp6p_tfg_moves(NULL, NULL, NULL);
		close_wnd_irp6p_tfg_servo_algorithm(NULL, NULL, NULL);

		close_wnd_speaker_play(NULL, NULL, NULL);

		execute_mp_pulse(MP_START);

		process_control_window_init(widget, apinfo, cbinfo);

		ui.manage_interface();
	}

	return (Pt_CONTINUE);

}

int pulse_stop_mp(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if ((ui.mp.state == UI_MP_TASK_RUNNING) || (ui.mp.state
			== UI_MP_TASK_PAUSED)) {

		ui.mp.state = UI_MP_WAITING_FOR_START_PULSE;// czekanie na stop

		execute_mp_pulse(MP_STOP);

		process_control_window_init(widget, apinfo, cbinfo);

		ui.manage_interface();
	}

	return (Pt_CONTINUE);

}

int pulse_pause_mp(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.mp.state == UI_MP_TASK_RUNNING) {

		ui.mp.state = UI_MP_TASK_PAUSED;// czekanie na stop

		execute_mp_pulse(MP_PAUSE);

		process_control_window_init(widget, apinfo, cbinfo);

		ui.manage_interface();
	}

	return (Pt_CONTINUE);

}

int pulse_resume_mp(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.mp.state == UI_MP_TASK_PAUSED) {

		ui.mp.state = UI_MP_TASK_RUNNING;// czekanie na stop

		execute_mp_pulse(MP_RESUME);

		process_control_window_init(widget, apinfo, cbinfo);

		ui.manage_interface();
	}

	return (Pt_CONTINUE);

}

int pulse_trigger_mp(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui.mp.state == UI_MP_TASK_RUNNING) {

		execute_mp_pulse(MP_TRIGGER);

		process_control_window_init(widget, apinfo, cbinfo);

		ui.manage_interface();
	}

	return (Pt_CONTINUE);

}

int execute_mp_pulse(char pulse_code) {
	int ret = -2;

	// printf("w send pulse\n");
	if (ui.mp.pulse_fd > 0) {
		long pulse_value = 1;
		if ((ret = MsgSendPulse(ui.mp.pulse_fd, sched_get_priority_min(
				SCHED_FIFO), pulse_code, pulse_value)) == -1) {

			perror("Blad w wysylaniu pulsu do mp");
			fprintf(stderr, "Blad w wysylaniu pulsu do mp error: %s \n",
					strerror(errno));
			delay(1000);
		}
	}
	return ret;

}

int signal_mp(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	fprintf(stderr, "UI: %s:%d signal_mp() -- thish should not happend!\n",
			__FILE__, __LINE__);
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
		ui.teachingstate = MP_PAUSED_H;
		// Zawieszenie procesow MP, ECP i EDP sygnalem SIGSTOP
	} else if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wnd_processes_control_signal_resume) {
		// Odwieszenie procesow EDP, ECP i MP sygnalem
		signo=SIGCONT;
		ui.teachingstate = MP_RUNNING;
	}

	/*int SignalKill( uint32_t nd,
	 pid_t pid,
	 int tid,
	 int signo,
	 int code,
	 int value );
	 */

	// if((	signo!=SIGCONT)&&(signo!=SIGSTOP)) {

	if ( ( ret = SignalKill(ui.irp6ot_m.state.edp.node_nr, ui.irp6ot_m.state.edp.pid, 1, signo,0,0) ) == -1 ) { // by Y !!! klopoty z wysylaniem do okreslonego watku -
		// wstawiona maska na odbior sygnalow po stronie serwo
		// 	perror("UI: Stop EDP failed");
		printf("sending a signal to edp failed\n");

	}
	// 	 }
	// XXX probably a bug - killing _ecp_ pid on _edp_ node (ptrojane)
	if ( ( ret = SignalKill(ui.irp6ot_m.state.edp.node_nr, ui.irp6ot_m.state.ecp.pid, 0, signo,0,0) ) == -1 ) {
		// 	perror("UI: Stop ECP failed");
		printf("sending a signal to ecp failed\n");

	}
	if ( ( ret = SignalKill(ui.mp.node_nr, ui.mp.pid, 0, signo,0,0) ) == -1 ) {
		// 	perror("UI: Stop MP failed");
		printf("sending a signal to mp failed\n");

	}

#endif
	return (Pt_CONTINUE);
}

int pulse_reader_all_robots_start(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	pulse_reader_irp6ot_start_exec_pulse();
	pulse_reader_irp6p_start_exec_pulse();
	pulse_reader_conv_start_exec_pulse();
	pulse_reader_irp6m_start_exec_pulse();

	process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_reader_all_robots_stop(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	pulse_reader_irp6ot_stop_exec_pulse();
	pulse_reader_irp6p_stop_exec_pulse();
	pulse_reader_conv_stop_exec_pulse();
	pulse_reader_irp6m_stop_exec_pulse();
	process_control_window_init(widget, apinfo, cbinfo);

	return (Pt_CONTINUE);

}

int pulse_reader_all_robots_trigger(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	pulse_reader_irp6ot_trigger_exec_pulse();
	pulse_reader_irp6p_trigger_exec_pulse();
	pulse_reader_conv_trigger_exec_pulse();
	pulse_reader_irp6m_trigger_exec_pulse();

	return (Pt_CONTINUE);

}

int pulse_reader_execute(int coid, int pulse_code, int pulse_value)

{

	if (MsgSendPulse(coid, sched_get_priority_min(SCHED_FIFO), pulse_code,
			pulse_value) == -1) {
		perror("Blad w wysylaniu pulsu do redera");
	}

	return 1;
}

int pulse_ecp_all_robots(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo)

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

bool check_node_existence(const std::string _node,
		const std::string beginnig_of_message) {

	std::string opendir_path("/net/");
	opendir_path += _node;

	if (access(opendir_path.c_str(), R_OK) != 0) {
		std::string tmp(beginnig_of_message);
		tmp += std::string(" node: ") + _node + std::string(" is unreachable");
		ui.ui_msg->message(lib::NON_FATAL_ERROR, tmp);

		return false;
	}
	return true;
}

