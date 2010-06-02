/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>// Y&7
#include <time.h>
#include <iostream>
#include <fstream>
#include <boost/circular_buffer.hpp>

#include "ui/ui.h"
#include "ui/ui_const.h"
#include "lib/srlib.h"
// #include "ecp/common/ecp.h"
#include "lib/com_buf.h"
#include "ui/ui_class.h"
#include "ui/src/ui_sr.h"
#include "ui/src/ui_ecp.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern busy_flag communication_flag;

ui_ecp_buffer::ui_ecp_buffer(Ui& _ui) :
	ui(_ui), communication_state(UI_ECP_AFTER_REPLY), synchroniser() {
	thread_id
			= new boost::thread(boost::bind(&ui_ecp_buffer::operator(), this));
}

ui_ecp_buffer::~ui_ecp_buffer() {
	//	printf("ui_ecp_buffer\n");
	//	thread_id->interrupt();
	//	thread_id->join(); // join it
	//	delete thread_id;
}

void ui_ecp_buffer::operator()() {

	lib::set_thread_priority(pthread_self(), MAX_PRIORITY - 5);

	lib::set_thread_name("comm");

	name_attach_t *attach;

	bool wyjscie;

	if ((attach = name_attach(NULL, ui.ui_attach_point.c_str(),
			NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		// TODO: throw
		// return EXIT_FAILURE;
		// printf("NIE MA ATTACHA");
	}

	while (1) {
		// ui.ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;
		ui.ui_ecp_obj->communication_state = UI_ECP_AFTER_REPLY;

		_msg_info info;

		int rcvid = MsgReceive(attach->chid, &ui.ui_ecp_obj->ecp_to_ui_msg,
				sizeof(ui.ui_ecp_obj->ecp_to_ui_msg), &info);

		ui.ui_ecp_obj->communication_state = UI_ECP_AFTER_RECEIVE;
		if (rcvid == -1) {/* Error condition, exit */
			perror("UI: Receive failed");
			// 	  throw generator::ECP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
			continue;
		}

		if (rcvid == 0) {/* Pulse received */
			// printf("sr puls\n");
			switch (ui.ui_ecp_obj->ecp_to_ui_msg.hdr.code) {
			case _PULSE_CODE_DISCONNECT:
				ConnectDetach(ui.ui_ecp_obj->ecp_to_ui_msg.hdr.scoid);
				break;
			case _PULSE_CODE_UNBLOCK:
				break;
			default:
				break;
			}
			continue;
		}

		/* A QNX IO message received, reject */
		if (ui.ui_ecp_obj->ecp_to_ui_msg.hdr.type >= _IO_BASE
				&& ui.ui_ecp_obj->ecp_to_ui_msg.hdr.type <= _IO_MAX) {

			MsgReply(rcvid, EOK, 0, 0);
			continue;
		}

		//! FIXME:
		if (ui.irp6ot_m->state.ecp.pid <= 0) {

			ui.irp6ot_m->state.ecp.pid = info.pid;

		}

		switch (ui.ui_ecp_obj->ecp_to_ui_msg.ecp_message) { // rodzaj polecenia z ECP
		case lib::C_XYZ_ANGLE_AXIS:
		case lib::C_XYZ_EULER_ZYZ:
		case lib::C_JOINT:
		case lib::C_MOTOR:
			//  printf("C_MOTOR\n");
			ui.ui_ecp_obj->synchroniser.null_command();
			if (ui.teachingstate == MP_RUNNING) {
				ui.teachingstate = ECP_TEACHING;
			}
			PtEnter(0);
			if (!ui.is_teaching_window_open) {
				ApCreateModule(ABM_teaching_window, ABW_base, NULL);
				ui.is_teaching_window_open = true;
			} else {
				PtWindowToFront(ABW_teaching_window);
			}
			PtLeave(0);
			ui.ui_ecp_obj->synchroniser.wait();

			if (MsgReply(rcvid, EOK, &ui.ui_ecp_obj->ui_rep,
					sizeof(ui.ui_ecp_obj->ui_rep)) < 0) {
				printf("Blad w UI reply\n");
			}
			break;
		case lib::YES_NO:
			ui.ui_ecp_obj->synchroniser.null_command();
			PtEnter(0);
			ApCreateModule(ABM_yes_no_window, ABW_base, NULL);
			PtSetResource(ABW_PtLabel_pytanie, Pt_ARG_TEXT_STRING,
					ui.ui_ecp_obj->ecp_to_ui_msg.string, 0);
			PtLeave(0);
			ui.ui_ecp_obj->synchroniser.wait();

			if (MsgReply(rcvid, EOK, &ui.ui_ecp_obj->ui_rep,
					sizeof(ui.ui_ecp_obj->ui_rep)) < 0) {
				printf("Blad w UI reply\n");
			}

			break;
		case lib::MESSAGE:
			PtEnter(0);
			ApCreateModule(ABM_wnd_message, ABW_base, NULL);
			PtSetResource(ABW_PtLabel_wind_message, Pt_ARG_TEXT_STRING,
					ui.ui_ecp_obj->ecp_to_ui_msg.string, 0);
			PtLeave(0);

			ui.ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;

			if (MsgReply(rcvid, EOK, &ui.ui_ecp_obj->ui_rep,
					sizeof(ui.ui_ecp_obj->ui_rep)) < 0) {
				printf("Blad w UI reply\n");
			}
			break;
		case lib::DOUBLE_NUMBER:
			ui.ui_ecp_obj->synchroniser.null_command();
			PtEnter(0);
			ApCreateModule(ABM_wnd_input_double, ABW_base, NULL);
			PtSetResource(ABW_PtLabel_wind_input_double, Pt_ARG_TEXT_STRING,
					ui.ui_ecp_obj->ecp_to_ui_msg.string, 0);
			PtLeave(0);
			ui.ui_ecp_obj->synchroniser.wait();

			if (MsgReply(rcvid, EOK, &ui.ui_ecp_obj->ui_rep,
					sizeof(ui.ui_ecp_obj->ui_rep)) < 0) {
				printf("Blad w UI reply\n");
			}
			break;
		case lib::INTEGER_NUMBER:
			ui.ui_ecp_obj->synchroniser.null_command();
			PtEnter(0);
			ApCreateModule(ABM_wnd_input_integer, ABW_base, NULL);
			PtSetResource(ABW_PtLabel_wind_input_integer, Pt_ARG_TEXT_STRING,
					ui.ui_ecp_obj->ecp_to_ui_msg.string, 0);
			PtLeave(0);
			ui.ui_ecp_obj->synchroniser.wait();

			if (MsgReply(rcvid, EOK, &ui.ui_ecp_obj->ui_rep,
					sizeof(ui.ui_ecp_obj->ui_rep)) < 0) {
				printf("Blad w UI reply\n");
			}
			break;
		case lib::CHOOSE_OPTION:
			ui.ui_ecp_obj->synchroniser.null_command();
			PtEnter(0);
			ApCreateModule(ABM_wnd_choose_option, ABW_base, NULL);
			PtSetResource(ABW_PtLabel_wind_choose_option, Pt_ARG_TEXT_STRING,
					ui.ui_ecp_obj->ecp_to_ui_msg.string, 0);

			// wybor ilosci dostepnych opcji w zaleznosci od wartosci ui.ui_ecp_obj->ecp_to_ui_msg.nr_of_options

			if (ui.ui_ecp_obj->ecp_to_ui_msg.nr_of_options == 2) {
				ui.block_widget(ABW_PtButton_wind_choose_option_3);
				ui.block_widget(ABW_PtButton_wind_choose_option_4);
			} else if (ui.ui_ecp_obj->ecp_to_ui_msg.nr_of_options == 3) {
				ui.unblock_widget(ABW_PtButton_wind_choose_option_3);
				ui.block_widget(ABW_PtButton_wind_choose_option_4);
			} else if (ui.ui_ecp_obj->ecp_to_ui_msg.nr_of_options == 4) {
				ui.unblock_widget(ABW_PtButton_wind_choose_option_3);
				ui.unblock_widget(ABW_PtButton_wind_choose_option_4);
			}

			PtLeave(0);
			ui.ui_ecp_obj->synchroniser.wait();

			if (MsgReply(rcvid, EOK, &ui.ui_ecp_obj->ui_rep,
					sizeof(ui.ui_ecp_obj->ui_rep)) < 0) {
				printf("Blad w UI reply\n");
			}

			break;
		case lib::LOAD_FILE: // Zaladowanie pliku - do ECP przekazywana jest nazwa pliku ze sciezka
			//    printf("lib::LOAD_FILE\n");
			if (ui.teachingstate == MP_RUNNING) {
				ui.ui_ecp_obj->synchroniser.null_command();
				wyjscie = false;
				while (!wyjscie) {
					if (!ui.is_file_selection_window_open) {
						ui.is_file_selection_window_open = 1;
						ui.file_window_mode = FSTRAJECTORY; // wybor pliku z trajektoria
						wyjscie = true;
						PtEnter(0);
						ApCreateModule(ABM_file_selection_window, ABW_base,
								NULL);
						// 	PtRealizeWidget( ABW_file_selection_window );
						PtLeave(0);
					} else {
						delay(1);
					}
				}

				ui.ui_ecp_obj->ui_rep.reply = lib::FILE_LOADED;
				ui.ui_ecp_obj->synchroniser.wait();

				if (MsgReply(rcvid, EOK, &ui.ui_ecp_obj->ui_rep,
						sizeof(ui.ui_ecp_obj->ui_rep)) < 0) {
					printf("Blad w UI reply\n");
				}

			}
			break;
		case lib::SAVE_FILE: // Zapisanie do pliku - do ECP przekazywana jest nazwa pliku ze sciezka
			//    printf("lib::SAVE_FILE\n");
			if (ui.teachingstate == MP_RUNNING) {
				ui.ui_ecp_obj->synchroniser.null_command();
				wyjscie = false;
				while (!wyjscie) {
					if (!ui.is_file_selection_window_open) {
						ui.is_file_selection_window_open = 1;
						ui.file_window_mode = FSTRAJECTORY; // wybor pliku z trajektoria
						wyjscie = true;
						PtEnter(0);
						ApCreateModule(ABM_file_selection_window, ABW_base,
								NULL);
						PtLeave(0);
					} else {
						delay(1);
					}
				}

				ui.ui_ecp_obj->ui_rep.reply = lib::FILE_SAVED;
				ui.ui_ecp_obj->synchroniser.wait();

				if (MsgReply(rcvid, EOK, &ui.ui_ecp_obj->ui_rep,
						sizeof(ui.ui_ecp_obj->ui_rep)) < 0) {
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
			ApCreateModule(ABM_MAM_wnd_manual_moves_automatic_measures,
					ABW_base, NULL);
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

}
