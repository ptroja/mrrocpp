#include <QTextCharFormat>
#include <QBrush>
#include <QColor>

#include <ctime>
#include <fstream>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "interface.h"
#include "ui_sr.h"

#include "irp6ot_m/ui_r_irp6ot_m.h"
#include "irp6p_m/ui_r_irp6p_m.h"
#include "spkm/ui_r_spkm.h"
#include "smb/ui_r_smb.h"
#include "shead/ui_r_shead.h"
#include "polycrank/ui_r_polycrank.h"
#include "spkm/wgt_spkm_inc.h"

MainWindow::MainWindow(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	QMainWindow(parent), ui(new Ui::MainWindow), interface(_interface)
{
	ui->setupUi(this);
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(on_timer_slot()));
	timer->start(50);

	connect(this, SIGNAL(ui_notification_signal(QString, QColor)), this, SLOT(ui_notification_slot(QString, QColor)), Qt::QueuedConnection);
	connect(this, SIGNAL(raise_process_control_window_signal()), this, SLOT(raise_process_control_window_slot()), Qt::QueuedConnection);

	// wyłączenie przycisku zamykania okna
	Qt::WindowFlags flags;
	flags |= Qt::WindowMaximizeButtonHint;
	flags |= Qt::WindowMinimizeButtonHint;
	setWindowFlags(flags);
}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::ui_notification(QString _string, QColor _color)
{
	//ui->notification_label->setText("GUGUGU");
	emit ui_notification_signal(_string, _color);
}

void MainWindow::raise_process_control_window()
{
	//ui->notification_label->setText("GUGUGU");
	emit raise_process_control_window_signal();
}

void MainWindow::raise_process_control_window_slot()
{
	interface.wgt_pc->dwgt->show();
	interface.wgt_pc->dwgt->raise();
	//	interface.dwgt_pc->setFloating(false);
	//interface.win_pc->show();
	//interface.win_pc->raise();
}

void MainWindow::ui_notification_slot(QString _string, QColor _color)
{
	QPalette pal;
	pal.setColor(QPalette::Text, _color);
	pal.setColor(QPalette::Foreground, _color);

	ui->notification_label->setPalette(pal);

	ui->notification_label->setText(_string);
}

void MainWindow::on_timer_slot()
{

	//fprintf(stderr, "OnTimer()\n");

	QTextCharFormat format;

	static int closing_delay_counter; // do odliczania czasu do zamkniecia aplikacji
	static int Iteration_counter = 0; // licznik uruchomienia funkcji


	Iteration_counter++;
	/* TR
	 if ((Iteration_counter % ui::common::CHECK_SPEAKER_STATE_ITER) == 0) {
	 if (interface.speaker->is_wind_speaker_play_open) // otworz okno
	 {
	 speaker_check_state(widget, apinfo, cbinfo);
	 }
	 }
	 */

	if (!(interface.ui_sr_obj->buffer_empty())) { // by Y jesli mamy co wypisywac

		// 	printf("timer\n");

		char current_line[400];
		lib::sr_package_t sr_msg;

		while (!(interface.ui_sr_obj->buffer_empty())) { // dopoki mamy co wypisywac

			interface.ui_sr_obj->get_one_msg(sr_msg);

			snprintf(current_line, 100, "%-10s", sr_msg.host_name);
			strcat(current_line, "  ");
			time_t time = sr_msg.time / 1000000000;
			strftime(current_line + 12, 100, "%H:%M:%S", localtime(&time));
			sprintf(current_line + 20, ".%03ld   ", (sr_msg.time % 1000000000) / 1000000);

			switch (sr_msg.process_type)
			{
				case lib::EDP:
					strcat(current_line, "edp: ");
					break;
				case lib::ECP:
					strcat(current_line, "ecp: ");
					break;
				case lib::MP:
					// printf("mp w ontimer\n");
					strcat(current_line, "mp:  ");
					break;
				case lib::VSP:
					strcat(current_line, "vsp: ");
					break;
				case lib::UI:
					strcat(current_line, "UI:  ");
					break;
				default:
					strcat(current_line, "???: ");
					continue;
			} // end: switch (message_buffer[reader_buf_position].process_type)

			// FIXME: ?
			sr_msg.process_type = lib::UNKNOWN_PROCESS_TYPE;

			char process_name_buffer[NAME_LENGTH + 1];
			snprintf(process_name_buffer, sizeof(process_name_buffer), "%-21s", sr_msg.process_name);

			strcat(current_line, process_name_buffer);

			switch (sr_msg.message_type)
			{
				case lib::FATAL_ERROR:
					strcat(current_line, "FATAL_ERROR:     ");
					format.setForeground(Qt::red);

					break;
				case lib::NON_FATAL_ERROR:

					strcat(current_line, "NON_FATAL_ERROR: ");
					format.setForeground(Qt::blue);

					break;
				case lib::SYSTEM_ERROR:
					// printf("SYSTEM ERROR W ONTIMER\n");
					// Informacja do UI o koniecznosci zmiany stanu na INITIAL_STATE
					strcat(current_line, "SYSTEM_ERROR:    ");
					format.setForeground(Qt::magenta);

					break;
				case lib::NEW_MESSAGE:
					strcat(current_line, "MESSAGE:         ");
					format.setForeground(Qt::black);

					break;
				default:
					strcat(current_line, "UNKNOWN ERROR:   ");
					format.setForeground(Qt::yellow);

			}; // end: switch (message.message_type)

			strcat(current_line, sr_msg.description);

			ui->plainTextEdit_sr->setCurrentCharFormat(format);
			ui->plainTextEdit_sr->appendPlainText(current_line);
			(*interface.log_file_outfile) << current_line;
		}

		(*interface.log_file_outfile).flush();

	}

	if (interface.ui_state == 2) {// jesli ma nastapic zamkniecie z aplikacji
		interface.set_ui_state_notification(UI_N_EXITING);
		// 	printf("w ontimer 2\n");
		closing_delay_counter = 20;// opoznienie zamykania
		interface.ui_state = 3;
		// 		delay(5000);

		interface.MPslay();

		interface.ui_msg->message("closing");
	} else if (interface.ui_state == 3) {// odliczanie
		// 	printf("w ontimer 3\n");
		if ((--closing_delay_counter) <= 0)
			interface.ui_state = 4;
	} else if (interface.ui_state == 4) {// jesli ma nastapic zamkniecie aplikacji
		//	printf("w ontimer 4\n");
		closing_delay_counter = 20;// opoznienie zamykania
		interface.ui_state = 5;

		interface.EDP_all_robots_slay();

	} else if (interface.ui_state == 5) {// odlcizanie do zamnkiecia
		//	printf("w ontimer 5\n");
		if ((--closing_delay_counter) <= 0)
			interface.ui_state = 6;
	} else if (interface.ui_state == 6) {// zakonczenie aplikacji
		(*interface.log_file_outfile).close();
		delete interface.log_file_outfile;
		printf("UI CLOSED\n");
		interface.abort_threads();
		interface.mw->close();
	} else {
		if (!(interface.communication_flag.is_busy())) {
			interface.set_ui_state_notification(UI_N_READY);
		}

	}

}

// menus

// file menu

void MainWindow::on_actionQuit_triggered()
{

	interface.UI_close();

}

// robot menu


// irp6ot_m menu

void MainWindow::on_actionirp6ot_m_EDP_Load_triggered()
{
	interface.irp6ot_m->edp_create();
}

void MainWindow::on_actionirp6ot_m_EDP_Unload_triggered()
{
	interface.irp6ot_m->EDP_slay_int();
}

void MainWindow::on_actionirp6ot_m_Synchronisation_triggered()
{
	interface.irp6ot_m->synchronise();
}

void MainWindow::on_actionirp6ot_m_Synchro_Position_triggered()
{
	interface.irp6ot_m->move_to_synchro_position();
}

void MainWindow::on_actionirp6ot_m_Front_Position_triggered()
{
	interface.irp6ot_m->move_to_front_position();
}

void MainWindow::on_actionirp6ot_m_Position_0_triggered()
{
	interface.irp6ot_m->move_to_preset_position(0);
}

void MainWindow::on_actionirp6ot_m_Position_1_triggered()
{
	interface.irp6ot_m->move_to_preset_position(1);
}

void MainWindow::on_actionirp6ot_m_Position_2_triggered()
{
	interface.irp6ot_m->move_to_preset_position(2);
}

// irp6p_m menu

void MainWindow::on_actionirp6p_m_EDP_Load_triggered()
{
	interface.irp6p_m->edp_create();
}

void MainWindow::on_actionirp6p_m_EDP_Unload_triggered()
{
	interface.irp6p_m->EDP_slay_int();
}

void MainWindow::on_actionirp6p_m_Synchronisation_triggered()
{
	interface.irp6p_m->synchronise();
}

void MainWindow::on_actionirp6p_m_Synchro_Position_triggered()
{
	interface.irp6p_m->move_to_synchro_position();
}

void MainWindow::on_actionirp6p_m_Front_Position_triggered()
{
	interface.irp6p_m->move_to_front_position();
}

void MainWindow::on_actionirp6p_m_Position_0_triggered()
{
	interface.irp6p_m->move_to_preset_position(0);
}

void MainWindow::on_actionirp6p_m_Position_1_triggered()
{
	interface.irp6p_m->move_to_preset_position(1);
}

void MainWindow::on_actionirp6p_m_Position_2_triggered()
{
	interface.irp6p_m->move_to_preset_position(2);
}

// spkm menu

void MainWindow::on_actionspkm_EDP_Load_triggered()
{
	interface.spkm->edp_create();
}

void MainWindow::on_actionspkm_EDP_Unload_triggered()
{
	interface.spkm->EDP_slay_int();
}

void MainWindow::on_actionspkm_Synchronisation_triggered()
{
	interface.spkm->synchronise();
}

void MainWindow::on_actionspkm_Motors_triggered()
{
	interface.spkm->wgt_inc->dwgt->show();
	interface.spkm->wgt_inc->dwgt->raise();
}

void MainWindow::on_actionspkm_Motors_post_triggered()
{
	interface.spkm->wgt_inc->dwgt->show();
	interface.spkm->wgt_inc->dwgt->raise();
}

void MainWindow::on_actionspkm_Joints_triggered()
{
	//	interface.spkm->wnd_int->show();
}

void MainWindow::on_actionspkm_External_triggered()
{
	//	interface.spkm->wnd_ext->show();
}

void MainWindow::on_actionspkm_Synchro_Position_triggered()
{
	interface.spkm->move_to_synchro_position();
}

void MainWindow::on_actionspkm_Front_Position_triggered()
{
	interface.spkm->move_to_front_position();
}

void MainWindow::on_actionspkm_Position_0_triggered()
{
	interface.spkm->move_to_preset_position(0);
}

void MainWindow::on_actionspkm_Position_1_triggered()
{
	interface.spkm->move_to_preset_position(1);
}

void MainWindow::on_actionspkm_Position_2_triggered()
{
	interface.spkm->move_to_preset_position(2);
}

// smb menu

void MainWindow::on_actionsmb_EDP_Load_triggered()
{
	interface.smb->edp_create();
}

void MainWindow::on_actionsmb_EDP_Unload_triggered()
{
	interface.smb->EDP_slay_int();
}

// shead menu

void MainWindow::on_actionshead_EDP_Load_triggered()
{
	interface.shead->edp_create();
}

void MainWindow::on_actionshead_EDP_Unload_triggered()
{
	interface.shead->EDP_slay_int();
}

// polycrank menu

void MainWindow::on_actionpolycrank_EDP_Load_triggered()
{
	interface.polycrank->edp_create();
}

void MainWindow::on_actionpolycrank_EDP_Unload_triggered()
{
	interface.polycrank->EDP_slay_int();
}

void MainWindow::on_actionpolycrank_Move_Joints_triggered()
{

}

// all robots menu


void MainWindow::on_actionall_EDP_Load_triggered()
{
	interface.EDP_all_robots_create();
}

void MainWindow::on_actionall_EDP_Uload_triggered()
{
	interface.EDP_all_robots_slay();
}

void MainWindow::on_actionall_Synchronisation_triggered()
{
	interface.EDP_all_robots_synchronise();
}

void MainWindow::on_actionall_Synchro_Position_triggered()
{
	interface.all_robots_move_to_synchro_position();
}

void MainWindow::on_actionall_Front_Position_triggered()
{
	interface.all_robots_move_to_front_position();
}

void MainWindow::on_actionall_Position_0_triggered()
{
	interface.all_robots_move_to_preset_position_0();
}

void MainWindow::on_actionall_Position_1_triggered()
{
	interface.all_robots_move_to_preset_position_1();
}

void MainWindow::on_actionall_Position_2_triggered()
{
	interface.all_robots_move_to_preset_position_2();
}

// task menu

void MainWindow::on_actionMP_Load_triggered()
{
	interface.MPup();
}

void MainWindow::on_actionMP_Unload_triggered()
{
	interface.MPslay();
}

void MainWindow::on_actionProcess_Control_triggered()
{
	raise_process_control_window();
}
void MainWindow::on_actionConfiguration_triggered()
{

}

// special menu


void MainWindow::on_actionClear_Console_triggered()
{
	ui->plainTextEdit_sr->clear();
}

void MainWindow::on_actionUnload_All_triggered()
{
	interface.unload_all();
}

void MainWindow::on_actionSlay_All_triggered()
{
	interface.slay_all();
}

