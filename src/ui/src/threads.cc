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

#include <boost/bind.hpp>
#include <boost/utility.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

#include <fcntl.h>
#include <string.h>
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

#include "lib/srlib.h"

pthread_t edp_irp6ot_tid;
pthread_t edp_irp6p_tid;
pthread_t edp_conv_tid;
pthread_t ui_tid;
pthread_t sr_tid;
pthread_t meb_tid;

function_execution_buffer edp_irp6ot_eb;
function_execution_buffer edp_irp6p_eb;
function_execution_buffer edp_conv_eb;
function_execution_buffer main_eb;

busy_flag communication_flag;

ui_sr_buffer* ui_sr_obj;

ui_ecp_buffer* ui_ecp_obj;

ui_msg_def ui_msg;

// forward declaration
void *sr_thread(void* arg);

#if !defined(USE_MESSIP_SRR)
/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"
#include <Pt.h>
#include <Ph.h>

extern ui_state_def ui_state;

void *comm_thread(void* arg)
{

	lib::set_thread_priority(pthread_self(), MAX_PRIORITY - 5);

	lib::set_thread_name("comm");

	name_attach_t *attach;

	_msg_info info;

	bool wyjscie;

	if ((attach = name_attach(NULL, ui_state.ui_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		// TODO: throw
		// return EXIT_FAILURE;
		// printf("NIE MA ATTACHA");
	}

	while (1) {
		// ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;
		ui_ecp_obj->communication_state = UI_ECP_AFTER_REPLY;

		int rcvid = MsgReceive(attach->chid, &ui_ecp_obj->ecp_to_ui_msg, sizeof(ui_ecp_obj->ecp_to_ui_msg), &info);

		ui_ecp_obj->communication_state = UI_ECP_AFTER_RECEIVE;
		if (rcvid == -1) {/* Error condition, exit */
			perror("UI: Receive failed");
			// 	  throw generator::ECP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
			continue;
		}

		if (rcvid == 0) {/* Pulse received */
			// printf("sr puls\n");
			switch (ui_ecp_obj->ecp_to_ui_msg.hdr.code)
			{
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
		if (ui_state.irp6_on_track.ecp.pid <= 0) {

			ui_state.irp6_on_track.ecp.pid = info.pid;

		}

		switch (ui_ecp_obj->ecp_to_ui_msg.ecp_message)
		{ // rodzaj polecenia z ECP
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
				if (!ui_state.is_teaching_window_open) {
					ApCreateModule(ABM_teaching_window, ABW_base, NULL);
					ui_state.is_teaching_window_open = true;
				} else {
					PtWindowToFront(ABW_teaching_window);
				}
				PtLeave(0);
				ui_ecp_obj->take_sem();

				if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep)) < 0) {
					printf("Blad w UI reply\n");
				}
				break;
			case lib::YES_NO:
				ui_ecp_obj->trywait_sem();
				PtEnter(0);
				ApCreateModule(ABM_yes_no_window, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_pytanie, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string , 0);
				PtLeave(0);
				ui_ecp_obj->take_sem();

				if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep)) < 0) {
					printf("Blad w UI reply\n");
				}

				break;
			case lib::MESSAGE:
				PtEnter(0);
				ApCreateModule(ABM_wnd_message, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_wind_message, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string , 0);
				PtLeave(0);

				ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;

				if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep)) < 0) {
					printf("Blad w UI reply\n");
				}
				break;
			case lib::DOUBLE_NUMBER:
				ui_ecp_obj->trywait_sem();
				PtEnter(0);
				ApCreateModule(ABM_wnd_input_double, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_wind_input_double, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string , 0);
				PtLeave(0);
				ui_ecp_obj->take_sem();

				if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep)) < 0) {
					printf("Blad w UI reply\n");
				}
				break;
			case lib::INTEGER_NUMBER:
				ui_ecp_obj->trywait_sem();
				PtEnter(0);
				ApCreateModule(ABM_wnd_input_integer, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_wind_input_integer, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string , 0);
				PtLeave(0);
				ui_ecp_obj->take_sem();

				if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep)) < 0) {
					printf("Blad w UI reply\n");
				}
				break;
			case lib::CHOOSE_OPTION:
				ui_ecp_obj->trywait_sem();
				PtEnter(0);
				ApCreateModule(ABM_wnd_choose_option, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_wind_choose_option, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string , 0);

				// wybor ilosci dostepnych opcji w zaleznosci od wartosci ui_ecp_obj->ecp_to_ui_msg.nr_of_options

				if (ui_ecp_obj->ecp_to_ui_msg.nr_of_options == 2) {
					block_widget(ABW_PtButton_wind_choose_option_3);
					block_widget(ABW_PtButton_wind_choose_option_4);
				} else if (ui_ecp_obj->ecp_to_ui_msg.nr_of_options == 3) {
					unblock_widget(ABW_PtButton_wind_choose_option_3);
					block_widget(ABW_PtButton_wind_choose_option_4);
				} else if (ui_ecp_obj->ecp_to_ui_msg.nr_of_options == 4) {
					unblock_widget(ABW_PtButton_wind_choose_option_3);
					unblock_widget(ABW_PtButton_wind_choose_option_4);
				}

				PtLeave(0);
				ui_ecp_obj->take_sem();

				if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep)) < 0) {
					printf("Blad w UI reply\n");
				}

				break;
			case lib::LOAD_FILE: // Zaladowanie pliku - do ECP przekazywana jest nazwa pliku ze sciezka
				//    printf("lib::LOAD_FILE\n");
				if (ui_state.teachingstate == MP_RUNNING) {
					ui_ecp_obj->trywait_sem();
					wyjscie = false;
					while (!wyjscie) {
						if (!ui_state.is_file_selection_window_open) {
							ui_state.is_file_selection_window_open = 1;
							ui_state.file_window_mode = FSTRAJECTORY; // wybor pliku z trajektoria
							wyjscie = true;
							PtEnter(0);
							ApCreateModule(ABM_file_selection_window, ABW_base, NULL);
							// 	PtRealizeWidget( ABW_file_selection_window );
							PtLeave(0);
						} else {
							delay(1);
						}
					}

					ui_ecp_obj->ui_rep.reply = lib::FILE_LOADED;
					ui_ecp_obj->take_sem();

					if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep)) < 0) {
						printf("Blad w UI reply\n");
					}

				}
				break;
			case lib::SAVE_FILE: // Zapisanie do pliku - do ECP przekazywana jest nazwa pliku ze sciezka
				//    printf("lib::SAVE_FILE\n");
				if (ui_state.teachingstate == MP_RUNNING) {
					ui_ecp_obj->trywait_sem();
					wyjscie = false;
					while (!wyjscie) {
						if (!ui_state.is_file_selection_window_open) {
							ui_state.is_file_selection_window_open = 1;
							ui_state.file_window_mode = FSTRAJECTORY; // wybor pliku z trajektoria
							wyjscie = true;
							PtEnter(0);
							ApCreateModule(ABM_file_selection_window, ABW_base, NULL);
							PtLeave(0);
						} else {
							delay(1);
						}
					}

					ui_ecp_obj->ui_rep.reply = lib::FILE_SAVED;
					ui_ecp_obj->take_sem();

					if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep)) < 0) {
						printf("Blad w UI reply\n");
					}
				}
				break;
			case lib::OPEN_FORCE_SENSOR_MOVE_WINDOW:
				// obsluga sterowania silowego -> ForceSensorMove
				// przejecie kontroli nad Fotonen

				PtEnter(0);
				// stworzenie okna wndForceControl
				ApCreateModule(ABM_wndForceControl, ABW_base, NULL);
				// 	oddanie kontroli
				PtLeave(0);
				// odeslanie -> odwieszenie ECP
				if (MsgReply(rcvid, EOK, NULL, 0) < 0) {
					printf("Blad w UI reply\n");
				}
				break;
			case lib::OPEN_TRAJECTORY_REPRODUCE_WINDOW:
				// obsluga odtwarzania trajektorii
				// przejecie kontroli nad Fotonen
				PtEnter(0);
				// stworzenie okna wndTrajectoryReproduce
				ApCreateModule(ABM_wndTrajectoryReproduce, ABW_base, NULL);
				// 	oddanie kontroli
				PtLeave(0);
				// odeslanie -> odwieszenie ECP
				if (MsgReply(rcvid, EOK, NULL, 0) < 0) {
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
				if (MsgReply(rcvid, EOK, NULL, 0) < 0) {
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
				if (MsgReply(rcvid, EOK, NULL, 0) < 0) {
					printf("Blad w UI reply\n");
				}
				break;

			case lib::MAM_OPEN_WINDOW:
				// Obsluga odtwarzania trajektorii.
				// Przejecie kontroli nad Fotonen.
				PtEnter(0);
				// Stworzenie okna wnd_manual_moves_automatic_measures.
				//		ApCreateModule (ABM_wndTrajectoryReproduce, ABW_base, NULL);
				ApCreateModule(ABM_MAM_wnd_manual_moves_automatic_measures, ABW_base, NULL);
				// Oddanie kontroli.
				PtLeave(0);
				// Odeslanie -> odwieszenie ECP.
				if (MsgReply(rcvid, EOK, NULL, 0) < 0) {
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
				if (MsgReply(rcvid, EOK, NULL, 0) < 0) {
					printf("Blad w UI reply\n");
				}
				break;

			default:
				perror("Strange ECP message");
		}; // end: switch
	}// end while

	return 0;
}

void *sr_thread(void* arg)
{
	lib::set_thread_name("sr");

	name_attach_t *attach;

	if ((attach = name_attach(NULL, ui_state.sr_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		perror("BLAD SR ATTACH, przypuszczalnie nie uruchomiono gns, albo blad wczytywania konfiguracji");
		return NULL;
	}

	while (1) {
		lib::sr_package_t sr_msg;
		//	printf("przed MsgReceive: \n");
		int rcvid = MsgReceive_r(attach->chid, &sr_msg, sizeof(sr_msg), NULL);
		//	printf("za MsgReceive: \n");
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
		if (sr_msg.hdr.type >= _IO_BASE && sr_msg.hdr.type <= _IO_MAX) {
			//  	  printf("w SR _IO_BASE _IO_MAX %d\n",_IO_CONNECT );
			//  MsgError(rcvid, ENOSYS);
			MsgReply(rcvid, EOK, 0, 0);
			continue;
		}

		int16_t status;
		MsgReply(rcvid, EOK, &status, sizeof(status));

		if (strlen(sr_msg.process_name) > 1) // by Y jesli ten string jest pusty to znaczy ze przyszedl smiec
		{
			// prrintf("srt: \n");
			flushall();

			ui_sr_obj->lock_mutex();

			//	ui_sr_obj->writer_buf_position++;
			//	ui_sr_obj->writer_buf_position %= UI_SR_BUFFER_LENGHT;

			// ui_sr_obj->message_buffer[ui_sr_obj->writer_buf_position] = sr_msg;
			ui_sr_obj->cb.push_back(sr_msg);
			ui_sr_obj->set_new_msg();
			ui_sr_obj->unlock_mutex();

		} else {
			printf("SR(%s:%d) unexpected message\n", __FILE__, __LINE__);
		}

	}

	return 0;
}
#endif /* USE_MESSIP_SRR */

void *edp_irp6ot_thread(void* arg)
{

	while (true) {
		edp_irp6ot_eb.wait_and_execute();
	}

	return NULL;
}

void *edp_irp6p_thread(void* arg)
{

	while (true) {
		edp_irp6p_eb.wait_and_execute();
	}

	return NULL;
}

void *edp_conv_thread(void* arg)
{

	while (true) {
		edp_conv_eb.wait_and_execute();
	}

	return NULL;
}

void *meb_thread(void* arg)
{

	while (true) {
		main_eb.wait_and_execute();
	}

	return NULL;
}

void create_threads()

{

	sigset_t set;

	sigemptyset(&set);
	sigaddset(&set, SIGINT);
	sigaddset(&set, SIGALRM);

	ui_sr_obj = new ui_sr_buffer();
	ui_ecp_obj = new ui_ecp_buffer();

	if (pthread_create(&sr_tid, NULL, sr_thread, NULL) != EOK) {// Y&W - utowrzenie watku serwa
		printf(" Failed to thread sr_thread\n");
	}
#if defined(__QNXNTO__)
	if (pthread_create(&ui_tid, NULL, comm_thread, NULL) != EOK) {// Y&W - utowrzenie watku serwa
		printf(" Failed to thread comm_thread\n");
	}
#endif
	if (pthread_create(&edp_irp6ot_tid, NULL, edp_irp6ot_thread, NULL) != EOK) {// Y&W - utowrzenie watku serwa
		printf(" Failed to thread edp_irp6ot_tid\n");
	}

	if (pthread_create(&edp_irp6p_tid, NULL, edp_irp6p_thread, NULL) != EOK) {// Y&W - utowrzenie watku serwa
		printf(" Failed to thread edp_irp6p_tid\n");
	}

	if (pthread_create(&edp_conv_tid, NULL, edp_conv_thread, NULL) != EOK) {// Y&W - utowrzenie watku serwa
		printf(" Failed to thread edp_conv_tid\n");
	}

	if (pthread_create(&meb_tid, NULL, meb_thread, NULL) != EOK) {// Y&W - utowrzenie watku serwa
		printf(" Failed to thread meb_tid\n");
	}

#if defined(__QNXNTO__)
	if (SignalProcmask(0, sr_tid, SIG_BLOCK, &set, NULL) == -1) {
		perror("SignalProcmask(sr_tid)");
	}

	if (SignalProcmask(0, ui_tid, SIG_BLOCK, &set, NULL) == -1) {
		perror("SignalProcmask(ui_tid)");
	}

	if (SignalProcmask(0, edp_irp6ot_tid, SIG_BLOCK, &set, NULL) == -1) {
		perror("SignalProcmask(edp_irp6ot_tid)");
	}

	if (SignalProcmask(0, edp_irp6p_tid, SIG_BLOCK, &set, NULL) == -1) {
		perror("SignalProcmask(edp_irp6p_tid)");
	}

	if (SignalProcmask(0, edp_conv_tid, SIG_BLOCK, &set, NULL) == -1) {
		perror("SignalProcmask(edp_conv_tid)");
	}

	if (SignalProcmask(0, meb_tid, SIG_BLOCK, &set, NULL) == -1) {
		perror("SignalProcmask(meb_tid)");
	}
#endif

}

void abort_threads()

{
#if defined(__QNXNTO__)
	pthread_abort(ui_tid);
	pthread_abort(sr_tid);
	pthread_abort(edp_irp6ot_tid);
	pthread_abort(edp_irp6p_tid);
	pthread_abort(edp_conv_tid);
	pthread_abort(meb_tid);
#endif
}

