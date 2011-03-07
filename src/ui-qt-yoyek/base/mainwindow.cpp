#include <QTextCharFormat>
#include <QBrush>
#include <QColor>
#include <QFileDialog>

#include <ctime>
#include <fstream>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "interface.h"
#include "ui_sr.h"

#include "../irp6ot_m/ui_r_irp6ot_m.h"
#include "../irp6p_m/ui_r_irp6p_m.h"
#include "../irp6p_tfg/ui_r_irp6p_tfg.h"
#include "../spkm/ui_r_spkm.h"
#include "../smb/ui_r_smb.h"
#include "../shead/ui_r_shead.h"
#include "../polycrank/ui_r_polycrank.h"
#include "../bird_hand/ui_r_bird_hand.h"
#include "../sarkofag/ui_r_sarkofag.h"
#include "../conveyor/ui_r_conveyor.h"

#include "../spkm/wgt_spkm_inc.h"
#include "../spkm/wgt_spkm_int.h"
#include "../spkm/wgt_spkm_ext.h"
#include "../polycrank/wgt_polycrank_int.h"
#include "../sarkofag/wgt_sarkofag_inc.h"
#include "../conveyor/wgt_conveyor_inc.h"
#include "../irp6p_tfg/wgt_irp6p_tfg_inc.h"

#include "../bird_hand/wgt_bird_hand_command.h"

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>

MainWindow::MainWindow(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	QMainWindow(parent), ui(new Ui::MainWindow), interface(_interface)
{
	ui->setupUi(this);
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(on_timer_slot()));
	timer->start(50);

	connect(this, SIGNAL(ui_notification_signal(QString, QColor)), this, SLOT(ui_notification_slot(QString, QColor)), Qt::QueuedConnection);
	connect(this, SIGNAL(raise_process_control_window_signal()), this, SLOT(raise_process_control_window_slot()), Qt::QueuedConnection);
	connect(this, SIGNAL(enable_menu_item_signal(QWidget *, bool)), this, SLOT(enable_menu_item_slot(QWidget *, bool)), Qt::QueuedConnection);
	connect(this, SIGNAL(enable_menu_item_signal(QAction *, bool)), this, SLOT(enable_menu_item_slot(QAction *, bool)), Qt::QueuedConnection);

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
Ui::MainWindow * MainWindow::get_ui()
{
	return ui;
}

//void MainWindow::enable_menu_item(bool _active, QWidget *_menu_item)
//{
//	interface.print_on_sr("signal");
//	emit enable_menu_item_signal(_menu_item, _active);
//}


void MainWindow::enable_menu_item(bool _enable, int _num_of_menus, QWidget *_menu_item, ...)
{
	va_list menu_items;

	emit
	enable_menu_item_signal(_menu_item, _enable);

	va_start(menu_items, _menu_item);

	for (int i = 1; i < _num_of_menus; i++) {
		//interface.print_on_sr("signal");
		emit enable_menu_item_signal(va_arg(menu_items, QWidget *), _enable);
	}

	va_end(menu_items);
}

void MainWindow::enable_menu_item(bool _enable, int _num_of_menus, QAction *_menu_item, ...)
{
	va_list menu_items;

	emit
	enable_menu_item_signal(_menu_item, _enable);

	va_start(menu_items, _menu_item);

	for (int i = 1; i < _num_of_menus; i++) {
		//interface.print_on_sr("signal");
		emit enable_menu_item_signal(va_arg(menu_items, QAction *), _enable);
	}

	va_end(menu_items);
}

//void MainWindow::disable_menu_item(int _num_of_menus, ...)
//{
//	va_list menu_items;
//	//QWidget *item;
//
//	va_start(menu_items, _num_of_menus);
//
//	for(int i=0; i<_num_of_menus; i++)
//	{
//	interface.print_on_sr("signal");
//	emit enable_menu_item_signal(va_arg(menu_items, QWidget *), false);
//	}
//
//	va_end(menu_items);
//}

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

void MainWindow::get_lineEdit_position(double* val, int number_of_servos)
{

	// TODO dodac obsluge wyjatku
	std::string text((ui->lineEdit_position->text()).toStdString());

	boost::char_separator <char> sep(" ");
	boost::tokenizer <boost::char_separator <char> > tokens(text, sep);

	int j = 0;
	BOOST_FOREACH(std::string t, tokens)
				{

					val[j] = boost::lexical_cast <double>(t);

					if (j == number_of_servos) {
						break;
					}
					j++;
				}

}

void MainWindow::raise_process_control_window_slot()
{
	interface.wgt_pc->my_open();
}

void MainWindow::enable_menu_item_slot(QWidget *_menu_item, bool _active)
{
	//interface.print_on_sr("menu coloring slot");
	_menu_item->setDisabled(!_active);
}

void MainWindow::enable_menu_item_slot(QAction *_menu_item, bool _active)
{
	//interface.print_on_sr("menu coloring slot");
	_menu_item->setDisabled(!_active);
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

	if (!(interface.ui_sr_obj->buffer_empty())) { // by Y jesli mamy co wypisywac

		// 	printf("timer\n");

		char current_line[400];
		lib::sr_package_t sr_msg;

		while (!(interface.ui_sr_obj->buffer_empty())) { // dopoki mamy co wypisywac

			interface.ui_sr_obj->get_one_msg(sr_msg);

			snprintf(current_line, 100, "%-10s", sr_msg.host_name);
			strcat(current_line, "  ");
			time_t time = sr_msg.tv.tv_sec;
			strftime(current_line + 12, 100, "%H:%M:%S", localtime(&time));
			sprintf(current_line + 20, ".%03u   ", (sr_msg.tv.tv_usec / 1000));

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
		interface.get_main_window()->close();
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

void MainWindow::on_actionirp6ot_m_Pre_Synchro_Moves_Motors_triggered()
{

}

void MainWindow::on_actionirp6ot_m_Absolute_Moves_Motors_triggered()
{

}

void MainWindow::on_actionirp6ot_m_Joints_triggered()
{

}

void MainWindow::on_actionirp6ot_m_Absolute_Moves_Xyz_Euler_Zyz_triggered()
{

}

void MainWindow::on_actionirp6ot_m_Absolute_Moves_Xyz_Angle_Axis_triggered()
{

}

void MainWindow::on_actionirp6ot_m_Relative_Xyz_Angle_Axis_triggered()
{

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

void MainWindow::on_actionirp6ot_m_Tool_Xyz_Euler_Zyz_triggered()
{

}

void MainWindow::on_actionirp6ot_m_Tool_Xyz_Angle_Axis_triggered()
{

}

//irp6ot_tfg

void MainWindow::on_actionirp6ot_tfg_EDP_Load_triggered()
{

}

void MainWindow::on_actionirp6ot_tfg_EDP_Unload_triggered()
{

}

void MainWindow::on_actionirp6ot_tfg_Synchronization_triggered()
{

}

void MainWindow::on_actionirp6ot_tfg_Move_triggered()
{

}

void MainWindow::on_actionirp6ot_tfg_Synchro_Position_triggered()
{

}

void MainWindow::on_actionirp6ot_tfg_Position_0_triggered()
{

}

void MainWindow::on_actionIrp6ot_tfg_Position_1_triggered()
{

}

void MainWindow::on_actionIrp6ot_tfg_Position_2_triggered()
{

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

void MainWindow::on_actionirp6p_m_Pre_Synchro_Moves_Motors_triggered()
{

}

void MainWindow::on_actionirp6p_m_Absolute_Moves_Motors_triggered()
{

}

void MainWindow::on_actionirp6p_m_Joints_triggered()
{

}

void MainWindow::on_actionirp6p_m_Absolute_Moves_Xyz_Euler_Zyz_triggered()
{

}

void MainWindow::on_actionirp6p_m_Absolute_Moves_Xyz_Angle_Axis_triggered()
{

}

void MainWindow::on_actionirp6p_m_Xyz_Relative_Moves_Angle_Axis_triggered()
{

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

void MainWindow::on_actionirp6p_m_Tool_Xyz_Euler_Zyz_triggered()
{

}

void MainWindow::on_actionirp6p_m_Tool_Xyz_Angle_Axis_triggered()
{

}

//irp6p_tfg

void MainWindow::on_actionirp6p_tfg_EDP_Load_triggered()
{
	interface.irp6p_tfg->edp_create();
}

void MainWindow::on_actionirp6p_tfg_EDP_Unload_triggered()
{
	interface.irp6p_tfg->EDP_slay_int();
}

void MainWindow::on_actionirp6p_tfg_Synchronization_triggered()
{
	interface.irp6p_tfg->synchronise();
}

void MainWindow::on_actionirp6p_tfg_Move_triggered()
{
	interface.irp6p_tfg->wgt_inc->my_open();
}

void MainWindow::on_actionirp6p_tfg_Synchro_Position_triggered()
{
	interface.irp6p_tfg->move_to_synchro_position();
}

void MainWindow::on_actionirp6p_tfg_Position_0_triggered()
{
	interface.irp6p_tfg->move_to_preset_position(0);
}

void MainWindow::on_actionirp6p_tfg_Position_1_triggered()
{
	interface.irp6p_tfg->move_to_preset_position(1);
}

void MainWindow::on_actionirp6p_tfg_Position_2_triggered()
{
	interface.irp6p_tfg->move_to_preset_position(2);
}

// conveyor menu
void MainWindow::on_actionconveyor_EDP_Load_triggered()
{
	interface.conveyor->edp_create();
}

void MainWindow::on_actionconveyor_EDP_Unload_triggered()
{
	interface.conveyor->EDP_slay_int();
}

void MainWindow::on_actionconveyor_Synchronization_triggered()
{
	interface.conveyor->synchronise();
}

void MainWindow::on_actionconveyor_Move_triggered()
{
	interface.conveyor->wgt_inc->my_open();
}

void MainWindow::on_actionconveyor_Synchro_Position_triggered()
{
	interface.conveyor->move_to_synchro_position();
}

void MainWindow::on_actionconveyor_Position_0_triggered()
{
	interface.conveyor->move_to_preset_position(0);
}

void MainWindow::on_actionconveyor_Position_1_triggered()
{
	interface.conveyor->move_to_preset_position(1);
}

void MainWindow::on_actionconveyor_Position_2_triggered()
{
	interface.conveyor->move_to_preset_position(2);
}

// birdhand menu
void MainWindow::on_actionbirdhand_EDP_Load_triggered()
{
	interface.bird_hand->edp_create();
}

void MainWindow::on_actionbirdhand_EDP_Unload_triggered()
{
	interface.bird_hand->EDP_slay_int();
}

void MainWindow::on_actionbirdhand_Command_triggered()
{
	interface.bird_hand->wnd_command_and_status->my_open();
}

void MainWindow::on_actionbirdhand_Configuration_triggered()
{

}

// sarkofag menu
void MainWindow::on_actionsarkofag_EDP_Load_triggered()
{
	interface.sarkofag->edp_create();
}

void MainWindow::on_actionsarkofag_EDP_Unload_triggered()
{
	interface.sarkofag->EDP_slay_int();
}

void MainWindow::on_actionsarkofag_Synchronisation_triggered()
{
	interface.sarkofag->synchronise();
}

void MainWindow::on_actionsarkofag_Move_triggered()
{
	interface.sarkofag->wgt_inc->my_open();
}

void MainWindow::on_actionsarkofag_Synchro_Position_triggered()
{
	interface.sarkofag->move_to_synchro_position();
}

void MainWindow::on_actionsarkofag_Front_Position_triggered()
{
	interface.sarkofag->move_to_front_position();
}

void MainWindow::on_actionsarkofag_Position_0_triggered()
{
	interface.sarkofag->move_to_preset_position(0);
}

void MainWindow::on_actionsarkofag_Position_1_triggered()
{
	interface.sarkofag->move_to_preset_position(1);
}

void MainWindow::on_actionsarkofag_Position_2_triggered()
{
	interface.sarkofag->move_to_preset_position(2);
}

void MainWindow::on_actionsarkofag_Servo_Algorithm_triggered()
{

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
	interface.spkm->wgt_inc->my_open();
}

void MainWindow::on_actionspkm_Motors_post_triggered()
{
	interface.spkm->wgt_inc->my_open();
}

void MainWindow::on_actionspkm_Joints_triggered()
{
	interface.spkm->wgt_int->my_open();
}

void MainWindow::on_actionspkm_External_triggered()
{
	interface.spkm->wgt_ext->my_open();
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

void MainWindow::on_actionspkm_Clear_Fault_triggered()
{
	interface.spkm->execute_clear_fault();
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

	interface.polycrank->wgt_int->my_open();
}

// all robots menu


void MainWindow::on_actionall_EDP_Load_triggered()
{
	interface.EDP_all_robots_create();
}

void MainWindow::on_actionall_EDP_Unload_triggered()
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
	/*
	 QFileDialog dialog;
	 if (dialog.exec()) {
	 // ...
	 }
	 */

	QString fileName;

	std::string mrrocpp_root_local_path =
			interface.mrrocpp_local_path.substr(0, interface.mrrocpp_local_path.rfind("/"));
	mrrocpp_root_local_path = mrrocpp_root_local_path.substr(0, mrrocpp_root_local_path.rfind("/") + 1);
	//interface.ui_msg->message(mrrocpp_root_local_path);

	try {

		fileName
				= QFileDialog::getOpenFileName(this, tr("Choose configuration file or die"), mrrocpp_root_local_path.c_str(), tr("Image Files (*.ini)"));

		std::string str_fullpath = fileName.toStdString();

		interface.config_file = str_fullpath.substr(str_fullpath.rfind(mrrocpp_root_local_path)
				+ mrrocpp_root_local_path.length());
		interface.reload_whole_configuration();
		interface.set_default_configuration_file_name();

	}

	catch (...) {

	}

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

