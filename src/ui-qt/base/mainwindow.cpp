#include <QTextCharFormat>
#include <QBrush>
#include <QColor>
#include <QFileDialog>
#include <QThread>

#include <ctime>
#include <fstream>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "ui.h"
#include "interface.h"
#include "ui_sr.h"
#include "ui_ecp.h"

#include "../irp6ot_m/ui_r_irp6ot_m.h"
#include "../irp6p_m/ui_r_irp6p_m.h"
#include "../irp6p_tfg/ui_r_irp6p_tfg.h"
#include "../irp6ot_tfg/ui_r_irp6ot_tfg.h"
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
#include "../base/wgt_single_motor_move.h"

#include "../bird_hand/wgt_bird_hand_command.h"

#include "../irp6_m/wgt_irp6_m_joints.h"
#include "../irp6_m/wgt_irp6_m_motors.h"
#include "../irp6_m/wgt_irp6_m_euler.h"
#include "../irp6_m/wgt_irp6_m_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_relative_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_tool_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_tool_euler.h"

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>

MainWindow::MainWindow(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	QMainWindow(parent), ui(new Ui::MainWindow), interface(_interface)
{
	ui->setupUi(this);
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(on_timer_slot()));
	timer->start(50);

	connect(this, SIGNAL(ui_notification_signal()), this, SLOT(ui_notification_slot()), Qt::QueuedConnection);
	connect(this, SIGNAL(raise_process_control_window_signal()), this, SLOT(raise_process_control_window_slot()), Qt::QueuedConnection);
	connect(this, SIGNAL(raise_ui_ecp_window_signal()), this, SLOT(raise_ui_ecp_window_slot()), Qt::QueuedConnection);
	connect(this, SIGNAL(enable_menu_item_signal(QWidget *, bool)), this, SLOT(enable_menu_item_slot(QWidget *, bool)), Qt::QueuedConnection);
	connect(this, SIGNAL(enable_menu_item_signal(QAction *, bool)), this, SLOT(enable_menu_item_slot(QAction *, bool)), Qt::QueuedConnection);

	main_thread_id = pthread_self();
}

void MainWindow::closeEvent(QCloseEvent *event)
{

	interface.UI_close();

	if (interface.ui_state == 6) {
		event->accept();
	} else {
		event->ignore();
	}
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
	// usuniete bo metoda wolana z dobrego watku przez manage interface_slot
	/*
	 emit
	 enable_menu_item_signal(_menu_item, _enable);
	 */
	enable_menu_item_slot(_menu_item, _enable);
	va_start(menu_items, _menu_item);

	for (int i = 1; i < _num_of_menus; i++) {
		//interface.print_on_sr("signal");
		// usuniete bo metoda wolana z dobrego watku przez manage interface_slot
		/*
		 emit enable_menu_item_signal(va_arg(menu_items, QWidget *), _enable);
		 */
		enable_menu_item_slot(va_arg(menu_items, QWidget *), _enable);
	}

	va_end(menu_items);
}

void MainWindow::enable_menu_item(bool _enable, int _num_of_menus, QAction *_menu_item, ...)
{
	va_list menu_items;
	// usuniete bo metoda wolana z dobrego watku przez manage interface_slot
	/*
	 emit
	 enable_menu_item_signal(_menu_item, _enable);
	 */
	enable_menu_item_slot(_menu_item, _enable);

	va_start(menu_items, _menu_item);

	for (int i = 1; i < _num_of_menus; i++) {
		//interface.print_on_sr("signal");
		// usuniete bo metoda wolana z dobrego watku przez manage interface_slot
		/*
		 emit enable_menu_item_signal(va_arg(menu_items, QAction *), _enable);
		 */
		enable_menu_item_slot(va_arg(menu_items, QAction *), _enable);
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

void MainWindow::ui_notification()
{

	if (main_thread_id == pthread_self()) {
		// jeśli wątek główny
		//	interface.ui_msg->message("same thread");
		ui_notification_slot();

	} else {
		//jeśli inny wątek niż główny
		//	interface.ui_msg->message("different thread");
		emit ui_notification_signal();
	}

}

void MainWindow::raise_process_control_window()
{
	//ui->notification_label->setText("GUGUGU");
	emit raise_process_control_window_signal();
}

void MainWindow::raise_ui_ecp_window()
{
	//ui->notification_label->setText("GUGUGU");
	emit raise_ui_ecp_window_signal();
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

void MainWindow::raise_ui_ecp_window_slot()
{
	interface.ui_msg->message("raise_ui_ecp_window_slot");

	lib::ECP_message &ecp_to_ui_msg = interface.ui_ecp_obj->ecp_to_ui_msg;
	lib::UI_reply &ui_rep = interface.ui_ecp_obj->ui_rep;

	switch (ecp_to_ui_msg.ecp_message)
	{ // rodzaj polecenia z ECP
		case lib::C_XYZ_ANGLE_AXIS: {
			if (interface.teachingstate == ui::common::MP_RUNNING) {
				interface.teachingstate = ui::common::ECP_TEACHING;
			}

			Ui::wgt_teachingClass* ui = interface.wgt_teaching_obj->get_ui();

			ui->label_message->setText("C_XYZ_ANGLE_AXIS");

			interface.wgt_teaching_obj->my_open();

			if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6ot_m::ROBOT_NAME) {
				/* TR
				 start_wnd_irp6_on_track_xyz_angle_axis(widget, apinfo, cbinfo);
				 */
			} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6p_m::ROBOT_NAME) {
				/* TR
				 start_wnd_irp6_postument_xyz_angle_axis(widget, apinfo, cbinfo);
				 */
			}

		}
			break;
		case lib::C_XYZ_EULER_ZYZ: {
			if (interface.teachingstate == ui::common::MP_RUNNING) {
				interface.teachingstate = ui::common::ECP_TEACHING;
			}

			Ui::wgt_teachingClass* ui = interface.wgt_teaching_obj->get_ui();

			ui->label_message->setText("C_XYZ_EULER_ZYZ");

			interface.wgt_teaching_obj->my_open();

			if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6ot_m::ROBOT_NAME) {
				/* TR
				 start_wnd_irp6_on_track_xyz_euler_zyz(widget, apinfo, cbinfo);
				 */
			} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6p_m::ROBOT_NAME) {
				/* TR
				 start_wnd_irp6_postument_xyz_euler_zyz(widget, apinfo, cbinfo);
				 */
			}

		}
			break;
		case lib::C_JOINT: {
			if (interface.teachingstate == ui::common::MP_RUNNING) {
				interface.teachingstate = ui::common::ECP_TEACHING;
			}

			Ui::wgt_teachingClass* ui = interface.wgt_teaching_obj->get_ui();

			ui->label_message->setText("C_JOINT");

			interface.wgt_teaching_obj->my_open();

			if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6ot_m::ROBOT_NAME) {
				interface.irp6ot_m->wgt_joints->my_open();
			} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6p_m::ROBOT_NAME) {
				interface.irp6p_m->wgt_joints->my_open();
			}

		}
			break;
		case lib::C_MOTOR: {
			//  printf("C_MOTOR\n");

			if (interface.teachingstate == ui::common::MP_RUNNING) {
				interface.teachingstate = ui::common::ECP_TEACHING;
			}

			Ui::wgt_teachingClass* ui = interface.wgt_teaching_obj->get_ui();

			ui->label_message->setText("C_MOTOR");

			interface.wgt_teaching_obj->my_open();

			if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6ot_m::ROBOT_NAME) {
				interface.irp6ot_m->wgt_motors->my_open();
			} else if (interface.ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6p_m::ROBOT_NAME) {
				interface.irp6p_m->wgt_motors->my_open();
			}

		}
			break;
		case lib::YES_NO: {
			Ui::wgt_yes_noClass* ui = interface.wgt_yes_no_obj->get_ui();

			ui->label_message->setText(ecp_to_ui_msg.string);

			interface.wgt_yes_no_obj->my_open();

		}
			break;
		case lib::MESSAGE: {
			Ui::wgt_messageClass* ui = interface.wgt_message_obj->get_ui();
			ui->label_message->setText(ecp_to_ui_msg.string);
			interface.wgt_message_obj->my_open();

			ui_rep.reply = lib::ANSWER_YES;
			interface.ui_ecp_obj->synchroniser.command();
		}
			break;
		case lib::DOUBLE_NUMBER: {

			Ui::wgt_input_doubleClass* ui = interface.wgt_input_double_obj->get_ui();

			ui->label_message->setText(ecp_to_ui_msg.string);

			interface.wgt_input_double_obj->my_open();
		}
			break;
		case lib::INTEGER_NUMBER: {

			Ui::wgt_input_integerClass* ui = interface.wgt_input_integer_obj->get_ui();

			ui->label_message->setText(ecp_to_ui_msg.string);

			interface.wgt_input_integer_obj->my_open();

		}
			break;
		case lib::CHOOSE_OPTION: {

			Ui::wgt_choose_optionClass* ui = interface.wgt_choose_option_obj->get_ui();

			ui->label_message->setText(ecp_to_ui_msg.string);

			// wybor ilosci dostepnych opcji w zaleznosci od wartosci ecp_to_ui_msg.nr_of_options

			if (ecp_to_ui_msg.nr_of_options == 2) {
				ui->pushButton_3->hide();
				ui->pushButton_4->hide();
			} else if (ecp_to_ui_msg.nr_of_options == 3) {
				ui->pushButton_3->show();
				ui->pushButton_4->hide();
			} else if (ecp_to_ui_msg.nr_of_options == 4) {
				ui->pushButton_3->show();
				ui->pushButton_4->show();
			}

			interface.wgt_choose_option_obj->my_open();
		}
			break;
		case lib::LOAD_FILE: {
			// Zaladowanie pliku - do ECP przekazywana jest nazwa pliku ze sciezka

			//    printf("lib::LOAD_FILE\n");


			interface.file_window_mode = ui::common::FSTRAJECTORY;

			try {
				QString fileName;

				fileName
						= QFileDialog::getOpenFileName(this, tr("Choose file to load or die"), interface.mrrocpp_root_local_path.c_str(), tr("Image Files (*)"));

				if (fileName.length() > 0) {

					strncpy(interface.ui_ecp_obj->ui_rep.filename, rindex(fileName.toStdString().c_str(), '/') + 1, strlen(rindex(fileName.toStdString().c_str(), '/'))
							- 1);
					interface.ui_ecp_obj->ui_rep.filename[strlen(rindex(fileName.toStdString().c_str(), '/')) - 1]
							= '\0';

					strncpy(interface.ui_ecp_obj->ui_rep.path, fileName.toStdString().c_str(), strlen(fileName.toStdString().c_str())
							- strlen(rindex(fileName.toStdString().c_str(), '/')));
					interface.ui_ecp_obj->ui_rep.path[strlen(fileName.toStdString().c_str())
							- strlen(rindex(fileName.toStdString().c_str(), '/'))] = '\0';

					ui_rep.reply = lib::FILE_LOADED;
				} else {
					ui_rep.reply = lib::QUIT;
				}
				//std::string str_fullpath = fileName.toStdString();
			}

			catch (...) {
				ui_rep.reply = lib::QUIT;
			}

			interface.ui_ecp_obj->synchroniser.command();

		}
			break;
		case lib::SAVE_FILE: {

			// Zapisanie do pliku - do ECP przekazywana jest nazwa pliku ze sciezka
			//    printf("lib::SAVE_FILE\n");

			interface.file_window_mode = ui::common::FSTRAJECTORY;

			try {
				QString fileName;

				fileName
						= QFileDialog::getSaveFileName(this, tr("Choose file to save or die"), interface.mrrocpp_root_local_path.c_str(), tr("Image Files (*)"));

				if (fileName.length() > 0) {

					strncpy(interface.ui_ecp_obj->ui_rep.filename, rindex(fileName.toStdString().c_str(), '/') + 1, strlen(rindex(fileName.toStdString().c_str(), '/'))
							- 1);
					interface.ui_ecp_obj->ui_rep.filename[strlen(rindex(fileName.toStdString().c_str(), '/')) - 1]
							= '\0';

					strncpy(interface.ui_ecp_obj->ui_rep.path, fileName.toStdString().c_str(), strlen(fileName.toStdString().c_str())
							- strlen(rindex(fileName.toStdString().c_str(), '/')));
					interface.ui_ecp_obj->ui_rep.path[strlen(fileName.toStdString().c_str())
							- strlen(rindex(fileName.toStdString().c_str(), '/'))] = '\0';

					ui_rep.reply = lib::FILE_SAVED;
				} else {
					ui_rep.reply = lib::QUIT;
				}
				//std::string str_fullpath = fileName.toStdString();
			}

			catch (...) {
				ui_rep.reply = lib::QUIT;
			}

			interface.ui_ecp_obj->synchroniser.command();

		}
			break;

		default: {
			perror("Strange ECP message");
			interface.ui_ecp_obj->synchroniser.command();
		}
			break;
	}

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

void MainWindow::ui_notification_slot()
{

	if (interface.next_notification != interface.notification_state) {

		QString _string;
		QColor _color;

		interface.notification_state = interface.next_notification;

		switch (interface.next_notification)
		{
			case UI_N_STARTING:
				_string = "STARTING";
				_color = Qt::magenta;

				break;
			case UI_N_READY:
				_string = "READY";
				_color = Qt::blue;

				break;
			case UI_N_BUSY:
				_string = "BUSY";
				_color = Qt::red;

				break;
			case UI_N_EXITING:
				_string = "EXITING";
				_color = Qt::magenta;

				break;
			case UI_N_COMMUNICATION:
				_string = "COMMUNICATION";
				_color = Qt::red;

				break;
			case UI_N_SYNCHRONISATION:
				_string = "SYNCHRONISATION";
				_color = Qt::red;

				break;
			case UI_N_PROCESS_CREATION:
				_string = "PROCESS CREATION";
				_color = Qt::red;

				break;
		}

		QPalette pal;
		pal.setColor(QPalette::Text, _color);
		pal.setColor(QPalette::Foreground, _color);

		ui->notification_label->setPalette(pal);

		ui->notification_label->setText(_string);
		ui->notification_label->repaint();
		ui->notification_label->update();
		qApp->processEvents();
	}
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
	interface.irp6ot_m->wgt_motors->my_open();
}

void MainWindow::on_actionirp6ot_m_Absolute_Moves_Motors_triggered()
{
	interface.irp6ot_m->wgt_motors->my_open();
}

void MainWindow::on_actionirp6ot_m_Joints_triggered()
{
	interface.irp6ot_m->wgt_joints->my_open();
}

void MainWindow::on_actionirp6ot_m_Absolute_Moves_Xyz_Euler_Zyz_triggered()
{
	interface.irp6ot_m->wgt_euler->my_open();
}

void MainWindow::on_actionirp6ot_m_Absolute_Moves_Xyz_Angle_Axis_triggered()
{
	interface.irp6ot_m->wgt_angle_axis->my_open();
}

void MainWindow::on_actionirp6ot_m_Relative_Xyz_Angle_Axis_triggered()
{
	interface.irp6ot_m->wgt_relative_angle_axis->my_open();
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
	interface.irp6ot_m->wgt_tool_euler->my_open();
}

void MainWindow::on_actionirp6ot_m_Tool_Xyz_Angle_Axis_triggered()
{
	interface.irp6ot_m->wgt_tool_angle_axis->my_open();
}

//irp6ot_tfg

void MainWindow::on_actionirp6ot_tfg_EDP_Load_triggered()
{
	interface.irp6ot_tfg->edp_create();
}

void MainWindow::on_actionirp6ot_tfg_EDP_Unload_triggered()
{
	interface.irp6ot_tfg->EDP_slay_int();
}

void MainWindow::on_actionirp6ot_tfg_Synchronization_triggered()
{
	interface.irp6ot_tfg->synchronise();
}

void MainWindow::on_actionirp6ot_tfg_Move_triggered()
{
	interface.irp6ot_tfg->wgt_move->my_open();
}

void MainWindow::on_actionirp6ot_tfg_Synchro_Position_triggered()
{
	interface.irp6ot_tfg->move_to_synchro_position();
}

void MainWindow::on_actionirp6ot_tfg_Position_0_triggered()
{
	interface.irp6ot_tfg->move_to_preset_position(0);
}

void MainWindow::on_actionirp6ot_tfg_Position_1_triggered()
{
	interface.irp6ot_tfg->move_to_preset_position(1);
}

void MainWindow::on_actionirp6ot_tfg_Position_2_triggered()
{
	interface.irp6ot_tfg->move_to_preset_position(2);
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
	interface.irp6p_m->wgt_motors->my_open();
}

void MainWindow::on_actionirp6p_m_Absolute_Moves_Motors_triggered()
{
	interface.irp6p_m->wgt_motors->my_open();
}

void MainWindow::on_actionirp6p_m_Joints_triggered()
{
	interface.irp6p_m->wgt_joints->my_open();
}

void MainWindow::on_actionirp6p_m_Absolute_Moves_Xyz_Euler_Zyz_triggered()
{
	interface.irp6p_m->wgt_euler->my_open();
}

void MainWindow::on_actionirp6p_m_Absolute_Moves_Xyz_Angle_Axis_triggered()
{
	interface.irp6p_m->wgt_angle_axis->my_open();
}

void MainWindow::on_actionirp6p_m_Xyz_Relative_Moves_Angle_Axis_triggered()
{
	interface.irp6p_m->wgt_relative_angle_axis->my_open();
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
	interface.irp6p_m->wgt_tool_euler->my_open();
}

void MainWindow::on_actionirp6p_m_Tool_Xyz_Angle_Axis_triggered()
{
	interface.irp6p_m->wgt_tool_angle_axis->my_open();
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
	interface.irp6p_tfg->wgt_move->my_open();
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
	interface.conveyor->wgt_move->my_open();
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
	interface.bird_hand->wgt_command_and_status->my_open();
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
	interface.sarkofag->wgt_move->my_open();
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
	try {
		QString fileName;

		std::string mrrocpp_current_config_full_path = interface.mrrocpp_root_local_path + interface.config_file;
		interface.ui_msg->message(mrrocpp_current_config_full_path);

		fileName
				= QFileDialog::getOpenFileName(this, tr("Choose configuration file or die"), mrrocpp_current_config_full_path.c_str(), tr("Image Files (*.ini)"));
		if (fileName.length() > 0) {
			std::string str_fullpath = fileName.toStdString();

			interface.config_file = str_fullpath.substr(str_fullpath.rfind(interface.mrrocpp_root_local_path)
					+ interface.mrrocpp_root_local_path.length());
			interface.reload_whole_configuration();
			interface.set_default_configuration_file_name();
		}
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

