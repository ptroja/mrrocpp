#include <gtk/gtk.h>
#include <cerrno>

#include "ui_model.h"

#include "base/lib/messip/messip_dataport.h"

#include "ui/src/ui.h"

extern "C" {
	void on_window_input_number_response(GtkDialog *dialog, gint arg1, gpointer user_data)
	{
		printf("response ID = %d\n", arg1);
	}
}

void *comm_thread(void* arg)
{
	messip_channel_t *ch;

	// TODO:interface.config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "ui_attach_point", lib::UI_SECTION)
	if ((ch = messip::port_create("ui")) == NULL) {
		return NULL;
	}

	ecp_buffer ui_ecp_obj;

	while (1) {
		// interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;
		//		interface.ui_ecp_obj->communication_state = UI_ECP_AFTER_REPLY;
		//		rcvid = MsgReceive(attach->chid, &interface.ui_ecp_obj->ecp_to_ui_msg, sizeof(interface.ui_ecp_obj->ecp_to_ui_msg), info);

		int32_t type, subtype;

		int rcvid = messip::port_receive(ch, type, subtype, ui_ecp_obj.ecp_to_ui_msg);
		//		interface.ui_ecp_obj->communication_state = UI_ECP_AFTER_RECEIVE;

		if (rcvid == -1) {/* Error condition, exit */
			perror("UI: Receive failed");
			// 	  throw ECP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
			break;
		} else if (rcvid < -1) {
			// channel open/close message
			continue;
		}

		switch (ui_ecp_obj.ecp_to_ui_msg.ecp_message)
		{
			case lib::DOUBLE_NUMBER: {
				gdk_threads_enter();
				GtkDialog *dialog = GTK_DIALOG(ui_model::instance().getUiGObject("window_input_number"));
				//		gtk_widget_show_all(GTK_WIDGET(dialog));
				gtk_widget_hide_on_delete(GTK_WIDGET(dialog));

				GtkSpinButton *input = GTK_SPIN_BUTTON((ui_model::instance().getUiGObject("numeric_input_spinbutton")));
				gtk_spin_button_set_range(input, -G_MAXDOUBLE, G_MAXDOUBLE);
				gtk_spin_button_set_value(input, 0.0);
				gtk_spin_button_set_digits(input, 3);

				GtkLabel *label = GTK_LABEL((ui_model::instance().getUiGObject("numeric_input_label")));
				gtk_label_set_label(label, ui_ecp_obj.ecp_to_ui_msg.string);

				gint response = gtk_dialog_run(dialog);
				gtk_widget_hide(GTK_WIDGET(dialog));
				gdk_flush();

				ui_ecp_obj.ui_rep.double_number = (response == GTK_RESPONSE_OK) ? lib::ANSWER_YES : lib::INVALID_REPLY;
				ui_ecp_obj.ui_rep.double_number = gtk_spin_button_get_value(input);

				gdk_threads_leave();

				if (messip::port_reply(ch, rcvid, EOK, ui_ecp_obj.ui_rep) < 0) {
					perror("messip::port_reply()");
				}
			}
			break;
			case lib::INTEGER_NUMBER: {
				gdk_threads_enter();
				GtkDialog *dialog = GTK_DIALOG(ui_model::instance().getUiGObject("window_input_number"));
				//		gtk_widget_show_all(GTK_WIDGET(dialog));
				gtk_widget_hide_on_delete(GTK_WIDGET(dialog));

				GtkSpinButton *input = GTK_SPIN_BUTTON((ui_model::instance().getUiGObject("numeric_input_spinbutton")));
				gtk_spin_button_set_range(input, -G_MAXINT, G_MAXINT);
				gtk_spin_button_set_value(input, 0.0);
				gtk_spin_button_set_digits(input, 0);

				GtkLabel *label = GTK_LABEL((ui_model::instance().getUiGObject("numeric_input_label")));
				gtk_label_set_label(label, ui_ecp_obj.ecp_to_ui_msg.string);

				gint response = gtk_dialog_run(dialog);
				gtk_widget_hide(GTK_WIDGET(dialog));
				gdk_flush();

				ui_ecp_obj.ui_rep.double_number = (response == GTK_RESPONSE_OK) ? lib::ANSWER_YES : lib::INVALID_REPLY;
				ui_ecp_obj.ui_rep.double_number = gtk_spin_button_get_value(input);

				gdk_threads_leave();

				if (messip::port_reply(ch, rcvid, EOK, ui_ecp_obj.ui_rep) < 0) {
					perror("messip::port_reply()");
				}
			}
			break;
#if 0
				// rodzaj polecenia z ECP
			case lib::C_XYZ_ANGLE_AXIS:
			case lib::C_XYZ_EULER_ZYZ:
			case lib::C_JOINT:
			case lib::C_MOTOR:
				//  printf("C_MOTOR\n");
				interface.ui_ecp_obj->synchroniser.null_command();
				if (interface.teachingstate == ui::common::MP_RUNNING) {
					interface.teachingstate = ECP_TEACHING;
				}
				PtEnter(0);
				if (!interface.is_teaching_window_open) {
					ApCreateModule(ABM_teaching_window, ABW_base, NULL);
					interface.is_teaching_window_open = true;
				} else {
					PtWindowToFront(ABW_teaching_window);
				}
				PtLeave(0);
				interface.ui_ecp_obj->synchroniser.wait();

				if (MsgReply(rcvid, EOK, &interface.ui_ecp_obj->ui_rep, sizeof(interface.ui_ecp_obj->ui_rep)) < 0) {
					printf("Blad w UI reply\n");
				}
				break;
			case lib::YES_NO:
				interface.ui_ecp_obj->synchroniser.null_command();
				PtEnter(0);
				ApCreateModule(ABM_yes_no_window, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_pytanie, Pt_ARG_TEXT_STRING, interface.ui_ecp_obj->ecp_to_ui_msg.string, 0);
				PtLeave(0);
				interface.ui_ecp_obj->synchroniser.wait();

				if (MsgReply(rcvid, EOK, &interface.ui_ecp_obj->ui_rep, sizeof(interface.ui_ecp_obj->ui_rep)) < 0) {
					printf("Blad w UI reply\n");
				}

				break;
			case MESSAGE:
				PtEnter(0);
				ApCreateModule(ABM_wnd_message, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_wind_message, Pt_ARG_TEXT_STRING, interface.ui_ecp_obj->ecp_to_ui_msg.string, 0);
				PtLeave(0);

				interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;

				if (MsgReply(rcvid, EOK, &interface.ui_ecp_obj->ui_rep, sizeof(interface.ui_ecp_obj->ui_rep)) < 0) {
					printf("Blad w UI reply\n");
				}
				break;
			case lib::CHOOSE_OPTION:
				interface.ui_ecp_obj->synchroniser.null_command();
				PtEnter(0);
				ApCreateModule(ABM_wnd_choose_option, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_wind_choose_option, Pt_ARG_TEXT_STRING, interface.ui_ecp_obj->ecp_to_ui_msg.string, 0);

				// wybor ilosci dostepnych opcji w zaleznosci od wartosci interface.ui_ecp_obj->ecp_to_ui_msg.nr_of_options

				if (interface.ui_ecp_obj->ecp_to_ui_msg.nr_of_options == 2) {
					interface.block_widget(ABW_PtButton_wind_choose_option_3);
					interface.block_widget(ABW_PtButton_wind_choose_option_4);
				} else if (interface.ui_ecp_obj->ecp_to_ui_msg.nr_of_options == 3) {
					interface.unblock_widget(ABW_PtButton_wind_choose_option_3);
					interface.block_widget(ABW_PtButton_wind_choose_option_4);
				} else if (interface.ui_ecp_obj->ecp_to_ui_msg.nr_of_options == 4) {
					interface.unblock_widget(ABW_PtButton_wind_choose_option_3);
					interface.unblock_widget(ABW_PtButton_wind_choose_option_4);
				}

				PtLeave(0);
				interface.ui_ecp_obj->synchroniser.wait();

				if (MsgReply(rcvid, EOK, &interface.ui_ecp_obj->ui_rep, sizeof(interface.ui_ecp_obj->ui_rep)) < 0) {
					printf("Blad w UI reply\n");
				}

				break;
			case lib::LOAD_FILE: // Zaladowanie pliku - do ECP przekazywana jest nazwa pliku ze sciezka
				//    printf("lib::LOAD_FILE\n");
				if (interface.teachingstate == ui::common::MP_RUNNING) {
					interface.ui_ecp_obj->synchroniser.null_command();
					bool wyjscie = false;
					while (!wyjscie) {
						if (!interface.is_file_selection_window_open) {
							interface.is_file_selection_window_open = 1;
							interface.file_window_mode = ui::common::FSTRAJECTORY; // wybor pliku z trajektoria
							wyjscie = true;
							PtEnter(0);
							ApCreateModule(ABM_file_selection_window, ABW_base, NULL);
							// 	PtRealizeWidget( ABW_file_selection_window );
							PtLeave(0);
						} else {
							delay(1);
						}
					}

					interface.ui_ecp_obj->ui_rep.reply = FILE_LOADED;
					interface.ui_ecp_obj->synchroniser.wait();

					if (MsgReply(rcvid, EOK, &interface.ui_ecp_obj->ui_rep, sizeof(interface.ui_ecp_obj->ui_rep)) < 0) {
						printf("Blad w UI reply\n");
					}

				}
				break;
			case lib::SAVE_FILE: // Zapisanie do pliku - do ECP przekazywana jest nazwa pliku ze sciezka
				//    printf("lib::SAVE_FILE\n");
				if (interface.teachingstate == ui::common::MP_RUNNING) {
					interface.ui_ecp_obj->synchroniser.null_command();
					bool wyjscie = false;
					while (!wyjscie) {
						if (!interface.is_file_selection_window_open) {
							interface.is_file_selection_window_open = 1;
							interface.file_window_mode = ui::common::FSTRAJECTORY; // wybor pliku z trajektoria
							wyjscie = true;
							PtEnter(0);
							ApCreateModule(ABM_file_selection_window, ABW_base, NULL);
							PtLeave(0);
						} else {
							delay(1);
						}
					}

					interface.ui_ecp_obj->ui_rep.reply = FILE_SAVED;
					interface.ui_ecp_obj->synchroniser.wait();

					if (MsgReply(rcvid, EOK, &interface.ui_ecp_obj->ui_rep, sizeof(interface.ui_ecp_obj->ui_rep)) < 0) {
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
#else
			default:
				perror("Strange ECP message");
		}

		messip::port_reply_nack(ch, rcvid);
#endif
	}// end while

	messip::port_delete(ch);

	return 0;
}
