#include <gtk/gtk.h>

#include "ui_model.h"

#include "messip/messip.h"

#include "ui/ui.h"

void *comm_thread(void* arg)
{
	messip_channel_t *ch;

	// TODO: config->return_attach_point_name(configurator::CONFIG_SERVER, "ui_attach_point", "[ui]")
	if ((ch = messip_channel_create(NULL, "ui", MESSIP_NOTIMEOUT, 0)) == NULL) {
		perror("messip_channel_create()");
		return NULL;
	}

	ui_ecp_buffer ui_ecp_obj = ui_ecp_buffer();

//	{
//		gdk_threads_enter();
//		GtkDialog *input = GTK_DIALOG(ui_model::instance().getUiGObject("window-input-number"));
//		gtk_widget_show_all(GTK_WIDGET(input));
//		gdk_threads_leave();
//	}

	while (1) {
		// ui_ecp_obj->communication_state = UI_ECP_REPLY_READY;
//		ui_ecp_obj->communication_state = UI_ECP_AFTER_REPLY;
//		rcvid = MsgReceive(attach->chid, &ui_ecp_obj->ecp_to_ui_msg, sizeof(ui_ecp_obj->ecp_to_ui_msg), info);

		int32_t type, subtype;

		int rcvid = messip_receive(ch, &type, &subtype, &ui_ecp_obj.ecp_to_ui_msg, sizeof(ui_ecp_obj.ecp_to_ui_msg), MESSIP_NOTIMEOUT);
//		ui_ecp_obj->communication_state = UI_ECP_AFTER_RECEIVE;

		if (rcvid == -1) {/* Error condition, exit */
			perror("UI: Receive failed\n");
			// 	  throw generator::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
			break;
		}

		switch (ui_ecp_obj.ecp_to_ui_msg.ecp_message) {
			case DOUBLE_NUMBER:
//				ui_ecp_obj->trywait_sem();
//				PtEnter(0);
//				ApCreateModule(ABM_wnd_input_double, ABW_base, NULL);
//				PtSetResource(ABW_PtLabel_wind_input_double, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string, 0);
//				PtLeave(0);
//				ui_ecp_obj->take_sem();
//
//				if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep)) < 0) {
//					printf("Blad w UI reply\n");
//				}
				break;
			case INTEGER_NUMBER:
//				ui_ecp_obj->trywait_sem();
//				PtEnter(0);
//				ApCreateModule(ABM_wnd_input_integer, ABW_base, NULL);
//				PtSetResource(ABW_PtLabel_wind_input_integer, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string, 0);
//				PtLeave(0);
//				ui_ecp_obj->take_sem();
//
//				if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep)) < 0) {
//					printf("Blad w UI reply\n");
//				}
				break;
#if 0
			// rodzaj polecenia z ECP
			case C_XYZ_ANGLE_AXIS:
			case C_XYZ_EULER_ZYZ:
			case C_JOINT:
			case C_MOTOR:
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
			case YES_NO:
				ui_ecp_obj->trywait_sem();
				PtEnter(0);
				ApCreateModule(ABM_yes_no_window, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_pytanie, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string, 0);
				PtLeave(0);
				ui_ecp_obj->take_sem();

				if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep)) < 0) {
					printf("Blad w UI reply\n");
				}

				break;
			case MESSAGE:
				PtEnter(0);
				ApCreateModule(ABM_wnd_message, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_wind_message, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string, 0);
				PtLeave(0);

				ui_ecp_obj->ui_rep.reply = ANSWER_YES;

				if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep)) < 0) {
					printf("Blad w UI reply\n");
				}
				break;
			case CHOOSE_OPTION:
				ui_ecp_obj->trywait_sem();
				PtEnter(0);
				ApCreateModule(ABM_wnd_choose_option, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_wind_choose_option, Pt_ARG_TEXT_STRING, ui_ecp_obj->ecp_to_ui_msg.string, 0);

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
			case LOAD_FILE: // Zaladowanie pliku - do ECP przekazywana jest nazwa pliku ze sciezka
				//    printf("LOAD_FILE\n");
				if (ui_state.teachingstate == MP_RUNNING) {
					ui_ecp_obj->trywait_sem();
					bool wyjscie = false;
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

					ui_ecp_obj->ui_rep.reply = FILE_LOADED;
					ui_ecp_obj->take_sem();

					if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep)) < 0) {
						printf("Blad w UI reply\n");
					}

				}
				break;
			case SAVE_FILE: // Zapisanie do pliku - do ECP przekazywana jest nazwa pliku ze sciezka
				//    printf("SAVE_FILE\n");
				if (ui_state.teachingstate == MP_RUNNING) {
					ui_ecp_obj->trywait_sem();
					bool wyjscie = false;
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

					ui_ecp_obj->ui_rep.reply = FILE_SAVED;
					ui_ecp_obj->take_sem();

					if (MsgReply(rcvid, EOK, &ui_ecp_obj->ui_rep, sizeof(ui_ecp_obj->ui_rep)) < 0) {
						printf("Blad w UI reply\n");
					}
				}
				break;
			case OPEN_FORCE_SENSOR_MOVE_WINDOW:
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
			case OPEN_TRAJECTORY_REPRODUCE_WINDOW:
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
			case TR_REFRESH_WINDOW:
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
			case TR_DANGEROUS_FORCE_DETECTED:
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
			case MAM_OPEN_WINDOW:
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
			case MAM_REFRESH_WINDOW:
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
#else
			default:
				perror("Strange ECP message\n");
		}; // end: switch

		messip_reply(ch, rcvid, EOK, NULL, 0, MESSIP_NOTIMEOUT);
#endif
	}// end while

	messip_channel_delete(ch, MESSIP_NOTIMEOUT);

	return 0;
}
