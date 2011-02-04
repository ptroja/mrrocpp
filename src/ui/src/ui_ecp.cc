/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <cerrno>// Y&7
#include <ctime>
#include <iostream>
#include <fstream>

#include "ui/src/ui.h"

#include "base/lib/sr/srlib.h"
#include "base/ecp/ecp_task.h"
#include "base/lib/com_buf.h"
#include "base/lib/mis_fun.h"
#include "ui/src/ui_class.h"
#include "ui/src/ui_sr.h"
#include "ui/src/ui_ecp.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

#include "base/lib/messip/messip_dataport.h"

namespace mrrocpp {

namespace ecp {
namespace common {
namespace task {

//! Dummy task required for dynamic linking
mrrocpp::ecp::common::task::task_base * return_created_ecp_task(mrrocpp::lib::configurator&)
{
	return NULL;
}

} // namespace ecp
} // namespace common
} // namespace task

namespace ui {
namespace common {

ecp_buffer::ecp_buffer(Interface& _interface) :
	interface(_interface), communication_state(UI_ECP_AFTER_REPLY), synchroniser()
{
	thread_id = boost::thread(boost::bind(&ecp_buffer::operator(), this));
}

ecp_buffer::~ecp_buffer()
{
	// thread_id.interrupt();
	// thread_id.join(); // join it
}

void ecp_buffer::operator()()
{
	lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 5);

	lib::set_thread_name("comm");
	bool wyjscie;

	lib::fd_server_t ch;

	ch = messip::port_create(interface.ui_attach_point);
	assert(ch);

	while (1) {
		// communication_state = ui::common::UI_ECP_REPLY_READY;
		communication_state = UI_ECP_AFTER_REPLY;

		int32_t type, subtype;
		int rcvid = messip::port_receive(ch, type, subtype, ecp_to_ui_msg);

		if ((rcvid != MESSIP_MSG_NOREPLY) && (rcvid != 0)) {
			continue;
		}

		/*
		 //! FIXME:
		 if (interface.irp6ot_m->state.ecp.pid <= 0) {

		 interface.irp6ot_m->state.ecp.pid = info.pid;

		 }
		 */
		switch (ecp_to_ui_msg.ecp_message)
		{ // rodzaj polecenia z ECP
			case lib::C_XYZ_ANGLE_AXIS:
			case lib::C_XYZ_EULER_ZYZ:
			case lib::C_JOINT:
			case lib::C_MOTOR:
				//  printf("C_MOTOR\n");
				synchroniser.null_command();
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
				synchroniser.wait();

				messip::port_reply(ch, rcvid, 0, ui_rep);

				break;
			case lib::YES_NO:
				synchroniser.null_command();
				PtEnter(0);
				ApCreateModule(ABM_yes_no_window, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_pytanie, Pt_ARG_TEXT_STRING,
						ecp_to_ui_msg.string, 0);
				PtLeave(0);
				synchroniser.wait();

				messip::port_reply(ch, rcvid, 0, ui_rep);

				break;
			case lib::MESSAGE:
				PtEnter(0);
				ApCreateModule(ABM_wnd_message, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_wind_message, Pt_ARG_TEXT_STRING,
						ecp_to_ui_msg.string, 0);
				PtLeave(0);

				ui_rep.reply = lib::ANSWER_YES;

				messip::port_reply(ch, rcvid, 0, ui_rep);

				break;
			case lib::DOUBLE_NUMBER:
				synchroniser.null_command();
				PtEnter(0);
				ApCreateModule(ABM_wnd_input_double, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_wind_input_double, Pt_ARG_TEXT_STRING,
						ecp_to_ui_msg.string, 0);
				PtLeave(0);
				synchroniser.wait();

				messip::port_reply(ch, rcvid, 0, ui_rep);

				break;
			case lib::INTEGER_NUMBER:
				synchroniser.null_command();
				PtEnter(0);
				ApCreateModule(ABM_wnd_input_integer, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_wind_input_integer, Pt_ARG_TEXT_STRING,
						ecp_to_ui_msg.string, 0);
				PtLeave(0);
				synchroniser.wait();

				messip::port_reply(ch, rcvid, 0, ui_rep);

				break;
			case lib::CHOOSE_OPTION:
				synchroniser.null_command();
				PtEnter(0);
				ApCreateModule(ABM_wnd_choose_option, ABW_base, NULL);
				PtSetResource(ABW_PtLabel_wind_choose_option, Pt_ARG_TEXT_STRING,
						ecp_to_ui_msg.string, 0);

				// wybor ilosci dostepnych opcji w zaleznosci od wartosci ecp_to_ui_msg.nr_of_options

				if (ecp_to_ui_msg.nr_of_options == 2) {
					interface.block_widget(ABW_PtButton_wind_choose_option_3);
					interface.block_widget(ABW_PtButton_wind_choose_option_4);
				} else if (ecp_to_ui_msg.nr_of_options == 3) {
					interface.unblock_widget(ABW_PtButton_wind_choose_option_3);
					interface.block_widget(ABW_PtButton_wind_choose_option_4);
				} else if (ecp_to_ui_msg.nr_of_options == 4) {
					interface.unblock_widget(ABW_PtButton_wind_choose_option_3);
					interface.unblock_widget(ABW_PtButton_wind_choose_option_4);
				}

				PtLeave(0);
				synchroniser.wait();

				messip::port_reply(ch, rcvid, 0, ui_rep);

				break;
			case lib::LOAD_FILE: // Zaladowanie pliku - do ECP przekazywana jest nazwa pliku ze sciezka
				//    printf("lib::LOAD_FILE\n");
				if (interface.teachingstate == ui::common::MP_RUNNING) {
					synchroniser.null_command();
					wyjscie = false;
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

					ui_rep.reply = lib::FILE_LOADED;
					synchroniser.wait();

					messip::port_reply(ch, rcvid, 0, ui_rep);

				}
				break;
			case lib::SAVE_FILE: // Zapisanie do pliku - do ECP przekazywana jest nazwa pliku ze sciezka
				//    printf("lib::SAVE_FILE\n");
				if (interface.teachingstate == ui::common::MP_RUNNING) {
					synchroniser.null_command();
					wyjscie = false;
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

					ui_rep.reply = lib::FILE_SAVED;
					synchroniser.wait();

					messip::port_reply(ch, rcvid, 0, ui_rep);

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

				messip::port_reply_ack(ch, rcvid);

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

				messip::port_reply_ack(ch, rcvid);

				break;
			case lib::TR_REFRESH_WINDOW:
				// przejecie kontroli nad Fotonen
				PtEnter(0);
				// Odswiezenie okna
				TRRefreshWindow(NULL, NULL, NULL);
				// 	oddanie kontroli
				PtLeave(0);
				// odeslanie -> odwieszenie ECP

				messip::port_reply_ack(ch, rcvid);

				break;
			case lib::TR_DANGEROUS_FORCE_DETECTED:
				// przejecie kontroli nad Fotonen
				PtEnter(0);
				// Ustawienie stanu przyciskow.
				TRDangerousForceDetected(NULL, NULL, NULL);
				// 	oddanie kontroli
				PtLeave(0);
				// odeslanie -> odwieszenie ECP

				messip::port_reply_ack(ch, rcvid);

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

				messip::port_reply_ack(ch, rcvid);

				break;
			case lib::MAM_REFRESH_WINDOW:
				// Przejecie kontroli nad Photonen.
				PtEnter(0);
				// Odswiezenie okna.
				MAM_refresh_window(NULL, NULL, NULL);
				// 	oddanie kontroli
				PtLeave(0);
				// Odeslanie -> odwieszenie ECP.

				messip::port_reply_ack(ch, rcvid);

				break;

			default:
				perror("Strange ECP message");
		}
	}

}

}
} //namespace ui
} //namespace mrrocpp
