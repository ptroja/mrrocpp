// ------------------------------------------------------------------------
// Proces:		UI
// Plik:			ui_init.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:
// Autor:		twiniarski/ ostatnie zmiany tkornuta
// Data:		14.03.2006
// ------------------------------------------------------------------------

/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <strings.h>

#include <string.h>
#include <signal.h>
#include <dirent.h>

#include <sys/dispatch.h>

#include <boost/bind.hpp>
#include <boost/utility.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>

#include <fcntl.h>
#include <string.h>
#include <process.h>
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <iostream>
#include <fstream>

#include <pthread.h>
#include <errno.h>

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "ui/ui_const.h"
#include "ui/ui.h"
#include "lib/configurator.h"
#include "lib/mis_fun.h"
#include "ui/ui_ecp.h"

#include "lib/messip/messip.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"
#include <Pt.h>
#include <Ph.h>

function_execution_buffer edp_irp6ot_eb;
function_execution_buffer edp_irp6p_eb;

ui_sr_buffer* ui_sr_obj;

ui_ecp_buffer* ui_ecp_obj;

// ini_configs* ini_con;
lib::configurator* config;

ui_state_def ui_state;
extern ui_robot_def ui_robot;

ui_msg_def ui_msg;

std::ofstream *log_file_outfile;

#if !defined(USE_MESSIP_SRR)
void *sr_thread(void* arg)
{
	lib::set_thread_name("sr");

	name_attach_t *attach;

	if ((attach = name_attach(NULL, ui_state.sr_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == NULL)
	{
		perror("BLAD SR ATTACH, przypuszczalnie nie uruchomiono gns, albo blad wczytywania konfiguracji");
		return NULL;
	}

	while(1)
	{
		lib::sr_package_t sr_msg;

		int rcvid = MsgReceive_r(attach->chid, &sr_msg, sizeof(sr_msg), NULL);

		if (rcvid < 0) /* Error condition, exit */
		{
			if (rcvid == -EINTR) {
				//fprintf(stderr, "MsgReceive_r() interrupted by signal\n");
				continue;
			}

			fprintf(stderr, "SR: Receive failed (%s)\n", strerror(-rcvid));
			// 	  throw generator::ECP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
			break;
		}

		if (rcvid == 0) /* Pulse received */
		{
			// printf("sr puls\n");
			switch (sr_msg.hdr.code)
			{
			case _PULSE_CODE_DISCONNECT:
				ConnectDetach(sr_msg.hdr.scoid);
				break;
			case _PULSE_CODE_UNBLOCK:
				break;
			default:
				break;
			}
			continue;
		}

		/* A QNX IO message received, reject */
		if (sr_msg.hdr.type >= _IO_BASE && sr_msg.hdr.type <= _IO_MAX)
		{
			//  	  printf("w SR _IO_BASE _IO_MAX %d\n",_IO_CONNECT );
			//  MsgError(rcvid, ENOSYS);
			MsgReply(rcvid, EOK, 0, 0);
			continue;
		}

		int16_t status;
		MsgReply(rcvid, EOK, &status, sizeof(status));

		if (strlen(sr_msg.process_name)>1) // by Y jesli ten string jest pusty to znaczy ze przyszedl smiec
		{

			ui_sr_obj->lock_mutex();

			ui_sr_obj->writer_buf_position++;
			ui_sr_obj->writer_buf_position %= UI_SR_BUFFER_LENGHT;

			ui_sr_obj->message_buffer[ui_sr_obj->writer_buf_position]=sr_msg;

			ui_sr_obj->set_new_msg();
			ui_sr_obj->unlock_mutex();

		} else {
			printf("SR(%s:%d) unexpected message\n", __FILE__, __LINE__);
		}

	}

	return 0;
}
#else /* USE_MESSIP_SRR */
#warning "use messip :)"
void *sr_thread(void* arg)
{
	messip_channel_t *ch;

	if ((ch = messip_channel_create(NULL, ui_state.sr_attach_point, MESSIP_NOTIMEOUT, 0)) == NULL) {
		return NULL;
	}

	while(1)
	{
		int32_t type, subtype;
		sr_package_t sr_msg;

		int rcvid = messip_receive(ch, &type, &subtype, &sr_msg, sizeof(sr_msg), MESSIP_NOTIMEOUT);

		if (rcvid == -1) /* Error condition, exit */
		{
			perror("SR: Receive failed");
			// 	  throw generator::ECP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
			break;
		} else if (rcvid < -1) {
			// ie. MESSIP_MSG_DISCONNECT
			fprintf(stderr, "ie. MESSIP_MSG_DISCONNECT\n");
			continue;
		}

		int16_t status = 0;
		messip_reply(ch, rcvid, EOK, &status, sizeof(status), MESSIP_NOTIMEOUT);

		if (strlen(sr_msg.process_name)>1) // by Y jesli ten string jest pusty to znaczy ze przyszedl smiec
		{
			ui_sr_obj->lock_mutex();
			// to sie zdarza choc nie wiem dlaczego

			ui_sr_obj->writer_buf_position++;
			ui_sr_obj->writer_buf_position %= UI_SR_BUFFER_LENGHT;

			ui_sr_obj->message_buffer[ui_sr_obj->writer_buf_position]=sr_msg;

			ui_sr_obj->set_new_msg();
			ui_sr_obj->unlock_mutex();

		} else {
			printf("SR(%s:%d) unexpected message\n", __FILE__, __LINE__);
		}
	}

	return 0;
}
#endif /* USE_MESSIP_SRR */


void *comm_thread(void* arg) {

	lib::set_thread_name("comm");

	name_attach_t *attach;

	_msg_info info;

	bool wyjscie;

	if ((attach = name_attach(NULL, ui_state.ui_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == NULL)
	{
		// TODO: throw
		// return EXIT_FAILURE;
		// printf("NIE MA ATTACHA");
	}

	while(1) {
		// ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;
		ui_ecp_obj->communication_state = UI_ECP_AFTER_REPLY;
		int rcvid = MsgReceive(attach->chid, &ui_ecp_obj->ecp_to_ui_msg, sizeof(ui_ecp_obj->ecp_to_ui_msg), &info);
		ui_ecp_obj->communication_state = UI_ECP_AFTER_RECEIVE;
		if (rcvid == -1) {/* Error condition, exit */
			perror("UI: Receive failed");
			// 	  throw generator::ECP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
			break;
		}

		if (rcvid == 0) {/* Pulse received */
			// printf("sr puls\n");
			switch (ui_ecp_obj->ecp_to_ui_msg.hdr.code) {
			case _PULSE_CODE_DISCONNECT:
				ConnectDetach(ui_ecp_obj->ecp_to_ui_msg.hdr.scoid);
				break;
			case _PULSE_CODE_UNBLOCK:
				break;
			default:
				break;
			}
			continue;
		}

		/* A QNX IO message received, reject */
		if (ui_ecp_obj->ecp_to_ui_msg.hdr.type >= _IO_BASE && ui_ecp_obj->ecp_to_ui_msg.hdr.type <= _IO_MAX) {

			MsgReply(rcvid, EOK, 0, 0);
			continue;
		}

		//! FIXME:
		if (ui_state.irp6_on_track.ecp.pid<=0) {

			ui_state.irp6_on_track.ecp.pid = info.pid;

		}

		switch ( ui_ecp_obj->ecp_to_ui_msg.ecp_message ) { // rodzaj polecenia z ECP
		case lib::C_XYZ_ANGLE_AXIS:
		case lib::C_XYZ_EULER_ZYZ:
		case lib::C_JOINT:
		case lib::C_MOTOR:
			//  printf("C_MOTOR\n");
			ui_ecp_obj->trywait_sem();
			if (ui_state.teachingstate == MP_RUNNING) {
				ui_state.teachingstate = ECP_TEACHING;
			}
			PtEnter(0);
			if(!ui_state.is_teaching_window_open) {
				ApCreateModule (ABM_teaching_window, ABW_base, NULL);
				ui_state.is_teaching_window_open=true;
			} else {
				PtWindowToFront (ABW_teaching_window);
			}
			PtLeave(0);
			ui_ecp_obj->take_sem();

			if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
				printf("Blad w UI reply\n");
			}
			break;
		case lib::YES_NO:
			ui_ecp_obj->trywait_sem();
			PtEnter(0);
			ApCreateModule (ABM_yes_no_window, ABW_base, NULL);
			PtSetResource(ABW_PtLabel_pytanie, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string , 0);
			PtLeave(0);
			ui_ecp_obj->take_sem();

			if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
				printf("Blad w UI reply\n");
			}

			break;
		case lib::MESSAGE:
			PtEnter(0);
			ApCreateModule (ABM_wnd_message, ABW_base, NULL);
			PtSetResource(ABW_PtLabel_wind_message, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string , 0);
			PtLeave(0);

			ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;

			if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
				printf("Blad w UI reply\n");
			}
			break;
		case lib::DOUBLE_NUMBER:
			ui_ecp_obj->trywait_sem();
			PtEnter(0);
			ApCreateModule (ABM_wnd_input_double, ABW_base, NULL);
			PtSetResource(ABW_PtLabel_wind_input_double, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string , 0);
			PtLeave(0);
			ui_ecp_obj->take_sem();

			if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
				printf("Blad w UI reply\n");
			}
			break;
		case lib::INTEGER_NUMBER:
			ui_ecp_obj->trywait_sem();
			PtEnter(0);
			ApCreateModule (ABM_wnd_input_integer, ABW_base, NULL);
			PtSetResource(ABW_PtLabel_wind_input_integer, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string , 0);
			PtLeave(0);
			ui_ecp_obj->take_sem();

			if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
				printf("Blad w UI reply\n");
			}
			break;
		case lib::CHOOSE_OPTION:
			ui_ecp_obj->trywait_sem();
			PtEnter(0);
			ApCreateModule (ABM_wnd_choose_option, ABW_base, NULL);
			PtSetResource(ABW_PtLabel_wind_choose_option, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string , 0);

			// wybor ilosci dostepnych opcji w zaleznosci od wartosci ui_ecp_obj->ecp_to_ui_msg.nr_of_options

			if (ui_ecp_obj->ecp_to_ui_msg.nr_of_options==2)
			{
				block_widget(ABW_PtButton_wind_choose_option_3);
				block_widget(ABW_PtButton_wind_choose_option_4);
			}
			else if (ui_ecp_obj->ecp_to_ui_msg.nr_of_options==3)
			{
				unblock_widget(ABW_PtButton_wind_choose_option_3);
				block_widget(ABW_PtButton_wind_choose_option_4);
			}
			else if (ui_ecp_obj->ecp_to_ui_msg.nr_of_options==4)
			{
				unblock_widget(ABW_PtButton_wind_choose_option_3);
				unblock_widget(ABW_PtButton_wind_choose_option_4);
			}

			PtLeave(0);
			ui_ecp_obj->take_sem();

			if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
				printf("Blad w UI reply\n");
			}

			break;
		case lib::LOAD_FILE: // Zaladowanie pliku - do ECP przekazywana jest nazwa pliku ze sciezka
			//    printf("lib::LOAD_FILE\n");
			if (ui_state.teachingstate == MP_RUNNING) {
				ui_ecp_obj->trywait_sem();
				wyjscie=false;
				while (!wyjscie)
				{
					if(!ui_state.is_file_selection_window_open)
					{
						ui_state.is_file_selection_window_open=1;
						ui_state.file_window_mode=FSTRAJECTORY;	// wybor pliku z trajektoria
						wyjscie = true;
						PtEnter(0);
						ApCreateModule (ABM_file_selection_window, ABW_base, NULL);
						// 	PtRealizeWidget( ABW_file_selection_window );
						PtLeave(0);
					} else {
						delay(1);
					}
				}

				ui_ecp_obj->ui_rep.reply = lib::FILE_LOADED;
				ui_ecp_obj->take_sem();

				if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
					printf("Blad w UI reply\n");
				}

			}
			break;
		case lib::SAVE_FILE: // Zapisanie do pliku - do ECP przekazywana jest nazwa pliku ze sciezka
			//    printf("lib::SAVE_FILE\n");
			if (ui_state.teachingstate == MP_RUNNING) {
				ui_ecp_obj->trywait_sem();
				wyjscie = false;
				while (!wyjscie)
				{
					if(!ui_state.is_file_selection_window_open)
					{
						ui_state.is_file_selection_window_open=1;
						ui_state.file_window_mode=FSTRAJECTORY;	// wybor pliku z trajektoria
						wyjscie = true;
						PtEnter(0);
						ApCreateModule (ABM_file_selection_window, ABW_base, NULL);
						PtLeave(0);
					} else {
						delay(1);
					}
				}

				ui_ecp_obj->ui_rep.reply = lib::FILE_SAVED;
				ui_ecp_obj->take_sem();

				if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep))<0) {
					printf("Blad w UI reply\n");
				}
			}
			break;
		case lib::OPEN_FORCE_SENSOR_MOVE_WINDOW:
			// obsluga sterowania silowego -> ForceSensorMove
			// przejecie kontroli nad Fotonen

			PtEnter(0);
			// stworzenie okna wndForceControl
			ApCreateModule (ABM_wndForceControl, ABW_base, NULL);
			// 	oddanie kontroli
			PtLeave(0);
			// odeslanie -> odwieszenie ECP
			if (MsgReply(rcvid, EOK, NULL, 0)<0) {
				printf("Blad w UI reply\n");
			}
			break;
		case lib::OPEN_TRAJECTORY_REPRODUCE_WINDOW:
			// obsluga odtwarzania trajektorii
			// przejecie kontroli nad Fotonen
			PtEnter(0);
			// stworzenie okna wndTrajectoryReproduce
			ApCreateModule (ABM_wndTrajectoryReproduce, ABW_base, NULL);
			// 	oddanie kontroli
			PtLeave(0);
			// odeslanie -> odwieszenie ECP
			if (MsgReply(rcvid, EOK, NULL, 0)<0) {
				printf("Blad w UI reply\n");
			}
			break;
		case lib::TR_REFRESH_WINDOW:
			// przejecie kontroli nad Fotonen
			PtEnter(0);
			// Odswiezenie okna
			TRRefreshWindow(NULL, NULL, NULL);
			// 	oddanie kontroli
			PtLeave(0);
			// odeslanie -> odwieszenie ECP
			if (MsgReply(rcvid, EOK, NULL, 0)<0) {
				printf("Blad w UI reply\n");
			}
			break;
		case lib::TR_DANGEROUS_FORCE_DETECTED:
			// przejecie kontroli nad Fotonen
			PtEnter(0);
			// Ustawienie stanu przyciskow.
			TRDangerousForceDetected(NULL, NULL, NULL);
			// 	oddanie kontroli
			PtLeave(0);
			// odeslanie -> odwieszenie ECP
			if (MsgReply(rcvid, EOK, NULL, 0)<0) {
				printf("Blad w UI reply\n");
			}
			break;


		case lib::MAM_OPEN_WINDOW:
			// Obsluga odtwarzania trajektorii.
			// Przejecie kontroli nad Fotonen.
			PtEnter(0);
			// Stworzenie okna wnd_manual_moves_automatic_measures.
			//		ApCreateModule (ABM_wndTrajectoryReproduce, ABW_base, NULL);
			ApCreateModule (ABM_MAM_wnd_manual_moves_automatic_measures, ABW_base, NULL);
			// Oddanie kontroli.
			PtLeave(0);
			// Odeslanie -> odwieszenie ECP.
			if (MsgReply(rcvid, EOK, NULL, 0)<0) {
				printf("Blad w UI reply\n");
			}
			break;
		case lib::MAM_REFRESH_WINDOW:
			// Przejecie kontroli nad Photonen.
			PtEnter(0);
			// Odswiezenie okna.
			MAM_refresh_window(NULL, NULL, NULL);
			// 	oddanie kontroli
			PtLeave(0);
			// Odeslanie -> odwieszenie ECP.
			if (MsgReply(rcvid, EOK, NULL, 0)<0) {
				printf("Blad w UI reply\n");
			}
			break;

		default:
			perror("Strange ECP message");
		}; // end: switch
	}// end while

	return 0;
};


/* Przechwycenie sygnalu */
void catch_signal(int sig) {
	int status;
	pid_t child_pid;

	// print a message
	fprintf(stderr, "UI: %s\n", strsignal(sig));

	switch(sig) {
	case SIGINT :
		UI_close();
		break;
	case SIGALRM:
		break;
	case SIGSEGV:
		signal(SIGSEGV, SIG_DFL);
		break;
	case SIGCHLD:
		child_pid = waitpid(-1, &status, 0);

		if (child_pid == -1) {
			int e = errno;
			perror("UI: waitpid()");
		} else if (child_pid == 0) {
			fprintf(stderr, "UI: no child exited\n");
		} else {
			//fprintf(stderr, "UI: child %d...\n", child_pid);
			if (WIFEXITED(status)) {
				fprintf(stderr, "UI: child %d exited normally with status %d\n",
						child_pid, WEXITSTATUS(status));
			}
			if (WIFSIGNALED(status)) {
#ifdef WCOREDUMP
				if (WCOREDUMP(status)) {
					fprintf(stderr, "UI: child %d terminated by signal %d (core dumped)\n",
							child_pid, WTERMSIG(status));
				}
				else
#endif /* WCOREDUMP */
				{
					fprintf(stderr, "UI: child %d terminated by signal %d\n",
							child_pid, WTERMSIG(status));
				}
			}
			if (WIFSTOPPED(status)) {
				fprintf(stderr, "UI: child %d stopped\n", child_pid);
			}
			if (WIFCONTINUED(status)) {
				fprintf(stderr, "UI: child %d resumed\n", child_pid);
			}
		}
		break;
	default:
		fprintf(stderr, "UI: unknown signal (%d)\n", sig);
	} // end: switch
}

void
UI_close(void) {
	printf("UI CLOSING\n");
	delay(100);// czas na ustabilizowanie sie edp
	ui_state.ui_state=2;// funcja OnTimer dowie sie ze aplikacja ma byc zamknieta
}




void *edp_irp6ot_thread(void* arg) {


	while(1)
	{
		edp_irp6ot_eb.wait();
		edp_irp6ot_eb.com_fun();
		printf("edp_irp6ot_thread: \n");
	}
	return 0;
}


void *edp_irp6p_thread(void* arg) {


	while(1)
	{
		edp_irp6p_eb.wait();
		edp_irp6p_eb.com_fun();
		printf("edp_irp6p_thread: \n");
	}
	return 0;
}



int init( PtWidget_t *link_instance, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

{
	/* eliminate 'unreferenced' warnings */
	link_instance = link_instance, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_STARTING);

	struct utsname sysinfo;
	char* cwd;
	char buff[PATH_MAX + 1];
	pthread_t ui_tid;
	pthread_t sr_tid;

	signal( SIGINT, &catch_signal );// by y aby uniemozliwic niekontrolowane zakonczenie aplikacji ctrl-c z kalwiatury
	signal( SIGALRM, &catch_signal );
	signal( SIGSEGV, &catch_signal );
#ifdef PROCESS_SPAWN_RSH
	signal( SIGCHLD, &catch_signal );
#endif /* PROCESS_SPAWN_RSH */

	lib::set_thread_priority(pthread_self() , MAX_PRIORITY-6);

	config = NULL;

	ui_state.ui_state=1;// ui working

	ui_state.irp6_on_track.edp.state=-1; // edp nieaktywne
	ui_state.irp6_on_track.edp.last_state=-1; // edp nieaktywne
	ui_state.irp6_on_track.ecp.trigger_fd = -1;
	ui_state.irp6_on_track.edp.section_name = "[edp_irp6_on_track]";
	ui_state.irp6_on_track.ecp.section_name = "[ecp_irp6_on_track]";
	ui_state.irp6_postument.edp.state=-1; // edp nieaktywne
	ui_state.irp6_postument.edp.last_state=-1; // edp nieaktywne
	ui_state.irp6_postument.ecp.trigger_fd = -1;
	ui_state.irp6_postument.edp.section_name = "[edp_irp6_postument]";
	ui_state.irp6_postument.ecp.section_name = "[ecp_irp6_postument]";
	ui_state.speaker.edp.state=-1; // edp nieaktywne
	ui_state.speaker.edp.last_state=-1; // edp nieaktywne
	ui_state.speaker.ecp.trigger_fd = -1;
	ui_state.speaker.edp.section_name = "[edp_speaker]";
	ui_state.speaker.ecp.section_name = "[ecp_speaker]";
	ui_state.conveyor.edp.state=-1; // edp nieaktywne
	ui_state.conveyor.edp.last_state=-1; // edp nieaktywne
	ui_state.conveyor.ecp.trigger_fd = -1;
	ui_state.conveyor.edp.section_name = "[edp_conveyor]";
	ui_state.conveyor.ecp.section_name = "[ecp_conveyor]";
	ui_state.irp6_mechatronika.edp.state=-1; // edp nieaktywne
	ui_state.irp6_mechatronika.edp.last_state=-1; // edp nieaktywne
	ui_state.irp6_mechatronika.ecp.trigger_fd = -1;
	ui_state.irp6_mechatronika.edp.section_name = "[edp_irp6_mechatronika]";
	ui_state.irp6_mechatronika.ecp.section_name = "[ecp_irp6_mechatronika]";

	ui_state.file_window_mode=FSTRAJECTORY; // uczenie
	ui_state.all_edps = UI_ALL_EDPS_NONE_EDP_LOADED;
	ui_state.mp.state = UI_MP_NOT_PERMITED_TO_RUN;// mp wylaczone
	ui_state.mp.last_state= UI_MP_NOT_PERMITED_TO_RUN ;// mp wylaczone
	ui_state.mp.pid = -1;
	ui_state.is_task_window_open=false;// informacja czy okno zadanai jest otwarte
	ui_state.is_process_control_window_open=false;// informacja czy okno sterowania procesami jest otwarte
	ui_state.process_control_window_renew = true;
	ui_state.is_file_selection_window_open=false;
	ui_state.is_wind_irp6ot_int_open=false;
	ui_state.is_wind_irp6p_int_open=false;
	ui_state.is_wind_irp6m_int_open=false;
	ui_state.is_wind_polycrank_int_open=false;
	ui_state.is_wind_irp6ot_inc_open=false;
	ui_state.is_wind_irp6p_inc_open=false;
	ui_state.is_wind_irp6m_inc_open=false;
	ui_state.is_wind_polycrank_inc_open=false;
	ui_state.is_wind_irp6ot_xyz_euler_zyz_open=false;
	ui_state.is_wind_irp6p_xyz_euler_zyz_open=false;
	ui_state.is_wind_irp6m_xyz_euler_zyz_open=false;
	ui_state.is_wind_irp6ot_xyz_angle_axis_open=false;
	ui_state.is_wind_irp6p_xyz_angle_axis_open=false;
	ui_state.is_wind_irp6m_xyz_angle_axis_open=false;
	ui_state.is_wind_irp6ot_xyz_angle_axis_ts_open=false;
	ui_state.is_wind_irp6p_xyz_angle_axis_ts_open=false;
	ui_state.is_wind_irp6m_xyz_angle_axis_ts_open=false;
	ui_state.is_wind_irp6ot_xyz_euler_zyz_ts_open=false;
	ui_state.is_wind_irp6p_xyz_euler_zyz_ts_open=false;
	ui_state.is_wind_irp6m_xyz_euler_zyz_ts_open=false;
	ui_state.is_teaching_window_open=false;
	ui_state.is_wind_conveyor_moves_open=false;
	ui_state.is_wind_irp6ot_kinematic_open=false;
	ui_state.is_wind_irp6p_kinematic_open=false;
	ui_state.is_wind_irp6m_kinematic_open=false;
	ui_state.is_wind_speaker_play_open=false;

	ui_state.is_wind_irp6ot_xyz_aa_relative_open=false;
	ui_state.is_wind_irp6p_xyz_aa_relative_open=false;

	ui_state.is_wind_irp6ot_servo_algorithm_open=false;
	ui_state.is_wind_irp6p_servo_algorithm_open=false;
	ui_state.is_wind_irp6m_servo_algorithm_open=false;
	ui_state.is_wind_conv_servo_algorithm_open=false;

	ui_state.is_mp_and_ecps_active = false;
	// ui_state.is_any_edp_active = false;


	ui_state.irp6_on_track.edp.is_synchronised = false;
	ui_state.irp6_postument.edp.is_synchronised = false;
	ui_state.conveyor.edp.is_synchronised = false;
	ui_state.speaker.edp.is_synchronised = false;
	ui_state.irp6_mechatronika.edp.is_synchronised = false;

	// ustalenie katalogow UI

	if( uname( &sysinfo ) == -1 ) {
		perror( "uname" );
	}

	cwd = getcwd( buff, PATH_MAX + 1 );
	if( cwd == NULL ) {
		perror( "Blad cwd w UI" );
	}

	ui_state.ui_node_name = sysinfo.nodename;

	ui_state.binaries_local_path = cwd;
	ui_state.mrrocpp_local_path = cwd;
	ui_state.mrrocpp_local_path.erase(ui_state.mrrocpp_local_path.length()-3);// kopiowanie lokalnej sciezki bez "bin" - 3 znaki
	ui_state.binaries_network_path = "/net/";
	ui_state.binaries_network_path += ui_state.ui_node_name;
	ui_state.binaries_network_path += ui_state.binaries_local_path;
	ui_state.binaries_network_path += "/";// wysylane jako argument do procesow potomnych (mp_m i dalej)
	// printf( "system name  : %s\n", ui_state.binaries_network_path);

	// sciezka dla okna z wyborem pliku podczas wybor trajektorii dla uczenia
	ui_state.teach_filesel_fullpath = "/net/";
	ui_state.teach_filesel_fullpath += ui_state.ui_node_name;
	ui_state.teach_filesel_fullpath += ui_state.mrrocpp_local_path;
	ui_state.teach_filesel_fullpath += "trj";
	// 	printf("abba: %s\n", ui_state.teach_filesel_fullpath);

	// sciezka dla okna z wyborem pliku z trajektoria podczas wyboru pliku konfiguracyjnego
	ui_state.config_file_fullpath = "/net/";
	ui_state.config_file_fullpath += ui_state.ui_node_name;
	ui_state.config_file_fullpath += ui_state.mrrocpp_local_path;
	ui_state.config_file_fullpath += "configs";

	// printf ("Remember to create gns server\n");

	// pierwsze zczytanie pliku konfiguracyjnego (aby pobrac nazwy dla pozostalych watkow UI)
	if (get_default_configuration_file_name()>=1) // zczytaj nazwe pliku konfiguracyjnego
	{
		initiate_configuration();
		// sprawdza czy sa postawione gns's i ew. stawia je
		// uwaga serwer musi byc wczesniej postawiony
		check_gns();
	} else {
		printf ("Blad manage_default_configuration_file\n");
		PtExit( EXIT_SUCCESS );
	}

	ui_sr_obj = new ui_sr_buffer();
	ui_ecp_obj = new ui_ecp_buffer();

	if (pthread_create (&sr_tid, NULL, sr_thread, NULL)!=EOK) {// Y&W - utowrzenie watku serwa
		printf (" Failed to thread sr_thread\n");
	}

	if (pthread_create (&ui_tid, NULL, comm_thread, NULL)!=EOK) {// Y&W - utowrzenie watku serwa
		printf (" Failed to thread comm_thread\n");
	}

	int edp_irp6ot_tid;

	if (pthread_create (&edp_irp6ot_tid, NULL, edp_irp6ot_thread, NULL)!=EOK) {// Y&W - utowrzenie watku serwa
		printf (" Failed to thread edp_irp6ot_tid\n");
	}

	int edp_irp6p_tid;

	if (pthread_create (&edp_irp6p_tid, NULL, edp_irp6p_thread, NULL)!=EOK) {// Y&W - utowrzenie watku serwa
		printf (" Failed to thread edp_irp6p_tid\n");
	}


	// Zablokowanie domyslnej obslugi sygnalu SIGINT w watkach UI_SR i UI_COMM

	sigset_t set;

	sigemptyset( &set );
	sigaddset( &set, SIGINT );
	sigaddset( &set, SIGALRM );

	if  (SignalProcmask(0, sr_tid, SIG_BLOCK, &set, NULL)==-1) {
		perror("SignalProcmask(sr_tid)");
	}

	if  (SignalProcmask(0, ui_tid, SIG_BLOCK, &set, NULL)==-1) {
		perror("SignalProcmask(ui_tid)");
	}

	// kolejne zczytanie pliku konfiguracyjnego
	if (get_default_configuration_file_name()==1) // zczytaj nazwe pliku konfiguracyjnego
	{
		reload_whole_configuration();

	} else {
		printf ("Blad manage_default_configuration_file\n");
		PtExit( EXIT_SUCCESS );
	}

	// inicjacja pliku z logami sr

	time_t time_of_day;
	char file_date[50];
	char log_file_with_dir[100];
	char file_name[50];

	time_of_day = time( NULL );
	strftime( file_date, 40, "%g%m%d_%H-%M-%S", localtime( &time_of_day ) );

	sprintf(file_name,"/%s_sr_log", file_date);

	// 	strcpy(file_name,"/pomiar.p");
	strcpy(log_file_with_dir, "../logs/");
	strcat(log_file_with_dir, file_name);

	log_file_outfile = new std::ofstream(log_file_with_dir, std::ios::out);

	if (!(*log_file_outfile)) {
		std::cerr << "Cannot open file: " << file_name << '\n';
		perror("because of");
	}

	manage_interface();

	return( Pt_CONTINUE );

}
