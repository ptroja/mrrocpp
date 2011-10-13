#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <strings.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/wait.h>

#include <map>
#include <string>
#include <iostream>

#include <boost/regex.hpp>
#include <boost/foreach.hpp>

#include <QtGui/QApplication>
#include <QFileDialog>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "interface.h"
#include "ui_sr.h"
#include "ui_ecp.h"
#include "base/lib/ping.h"

#include "../spkm/ui_r_spkm1.h"
#include "../spkm/ui_r_spkm2.h"
#include "../smb/ui_r_smb1.h"
#include "../smb/ui_r_smb2.h"
#include "../shead/ui_r_shead1.h"
#include "../shead/ui_r_shead2.h"
#include "../sbench/ui_r_sbench.h"
#include "../irp6ot_m/ui_r_irp6ot_m.h"
#include "../irp6p_m/ui_r_irp6p_m.h"
#include "../irp6p_tfg/ui_r_irp6p_tfg.h"
#include "../irp6ot_tfg/ui_r_irp6ot_tfg.h"
#include "../polycrank/ui_r_polycrank.h"
#include "../bird_hand/ui_r_bird_hand.h"
#include "../sarkofag/ui_r_sarkofag.h"
#include "../conveyor/ui_r_conveyor.h"

#include "../irp6_m/wgt_irp6_m_joints.h"
#include "../irp6_m/wgt_irp6_m_motors.h"
#include "../irp6_m/wgt_irp6_m_euler.h"
#include "../irp6_m/wgt_irp6_m_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_relative_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_tool_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_tool_euler.h"

#include "wgt_robot_process_control.h"
#include "mp.h"
#include "allrobots.h"

extern void catch_signal(int sig);

namespace mrrocpp {
namespace ui {
namespace common {

Interface::Interface() :
		is_mp_and_ecps_active(false), position_refresh_interval(200), mrrocpp_bin_to_root_path("../../")
{

	mw = (boost::shared_ptr <MainWindow>) new MainWindow(*this);

	main_eb = new function_execution_buffer(*this);

	timer = (boost::shared_ptr <QTimer>) new QTimer(this);

	connect(timer.get(), SIGNAL(timeout()), this, SLOT(timer_slot()));
	connect(this, SIGNAL(manage_interface_signal()), this, SLOT(manage_interface_slot()), Qt::QueuedConnection);
	connect(this, SIGNAL(raise_process_control_window_signal()), this, SLOT(raise_process_control_window_slot()), Qt::QueuedConnection);
	connect(this, SIGNAL(raise_ui_ecp_window_signal()), this, SLOT(raise_ui_ecp_window_slot()), Qt::QueuedConnection);

	ui_state = 1; // ui working
	file_window_mode = ui::common::FSTRAJECTORY; // uczenie

	all_robots = new AllRobots(this);
	mp = new Mp(this);
}

Interface::~Interface()
{
	BOOST_FOREACH(robot_pair_t node, robot_m)
			{
				delete node.second;
			}
}

void Interface::start_on_timer()
{
	timer->start(50);
}

bool Interface::html_it(std::string &_input, std::string &_output)
{

	try {
		// Wyrażenie regularne reprezentujące pierwsze dwie kolumny (druga może być pusta)
		boost::regex pattern("(<)|(>)|( )|(&)");
		// Format stringu odpowiadający podmienianemu dopasowaniu.
		std::string fmt("(?1&lt;)(?2&gt;)(?3&#160;)(?4&amp;)");

		std::ostringstream t(std::ios::out | std::ios::binary);
		std::ostream_iterator <char> oi(t);
		boost::regex_merge(oi, _input.begin(), _input.end(), pattern, fmt, boost::match_default | boost::format_all);

		_output = t.str();

	} catch (std::exception &ex) {
		std::cout << "blad" << ex.what() << std::endl;
	}

	return true;
}

void Interface::timer_slot()
{

	//fprintf(stderr, "OnTimer()\n");

	QTextCharFormat format;

	static int closing_delay_counter; // do odliczania czasu do zamkniecia aplikacji
	static int Iteration_counter = 0; // licznik uruchomienia funkcji

	Iteration_counter++;

	if (!(ui_sr_obj->buffer_empty())) { // by Y jesli mamy co wypisywac

		// 	printf("timer\n");
		ui_sr_obj->copy_buffers();

		ui_sr_obj->clear_buffer();

		char current_line[400];

		std::string html_line;

		lib::sr_package_t sr_msg;
		int iterator = sr_buffer::UI_SR_BUFFER_LENGHT;

		while ((!(ui_sr_obj->inter_buffer_empty())) && ((iterator--) > 0)) { // dopoki mamy co wypisywac

			if (iterator < 2) {
				std::cout << "UI - bufor sr przepelniony" << std::endl;
			}
			ui_sr_obj->inter_get_one_msg(sr_msg);

			snprintf(current_line, 100, "%-10s", sr_msg.host_name);

			strcat(current_line, "  ");
			time_t time = sr_msg.tv.tv_sec;
			strftime(current_line + 12, 100, "%H:%M:%S", localtime(&time));
			sprintf(current_line + 20, ".%03u   ", (sr_msg.tv.tv_usec / 1000));

			std::string input(current_line);

			std::string output;

			html_it(input, output);

			html_line = "<font face=\"Monospace\" color=\"black\">" + output
					+ "</font><font face=\"Monospace\" color=\"";
			switch (sr_msg.process_type)
			{
				case lib::EDP:

					strcat(current_line, "D: ");
					html_line += "#767639\">D:&#160;";
					break;
				case lib::ECP:
					strcat(current_line, "C: ");
					html_line += "Slate Blue\">C:&#160;";
					break;
				case lib::MP:
					// printf("mp w ontimer\n");
					strcat(current_line, "M: ");
					html_line += "#a54e8f\">M:&#160;";
					break;
				case lib::VSP:
					strcat(current_line, "S: ");
					html_line += "brown\">S:&#160;";
					break;
				case lib::UI:
					strcat(current_line, "I: ");
					html_line += "brown\">I:&#160;";
					break;
				default:
					strcat(current_line, "?: ");
					html_line += "magenta\">?:&#160;";
					continue;
			} // end: switch (message_buffer[reader_buf_position].process_type)
			html_line += "</font>";
			// FIXME: ?
			sr_msg.process_type = lib::UNKNOWN_PROCESS_TYPE;

			char process_name_buffer[NAME_LENGTH + 1];snprintf
			(process_name_buffer, sizeof(process_name_buffer), "%-15s", sr_msg.process_name);

			strcat(current_line, process_name_buffer);

			input = std::string(process_name_buffer);

			html_it(input, output);

			html_line += "<font face=\"Monospace\" color=\"black\">" + output
					+ "</font><font face=\"Monospace\" color=\"";

			switch (sr_msg.message_type)
			{
				case lib::FATAL_ERROR:
					strcat(current_line, "FE:   ");
					//	format.setForeground(Qt::red);
					html_line += "black\" style=\"background-color:'#ffc6c6';\">FE:&#160;&#160;&#160;";
					break;
				case lib::NON_FATAL_ERROR:
					strcat(current_line, "NFE:  ");
					//	format.setForeground(Qt::blue);
					html_line += "black\" style=\"background-color:'#c6e7ff';\">NFE:&#160;&#160;";
					break;
				case lib::SYSTEM_ERROR:
					// printf("SYSTEM ERROR W ONTIMER\n");
					// Informacja do UI o koniecznosci zmiany stanu na INITIAL_STATE
					strcat(current_line, "SE:   ");
					//	format.setForeground(Qt::magenta);
					html_line += "black\" style=\"background-color:'#ffc6ef';\">SE:&#160;&#160;&#160;";

					break;
				case lib::NEW_MESSAGE:
					strcat(current_line, "MSG:  ");
					//	format.setForeground(Qt::black);
					html_line += "black\">msg:&#160;&#160;";
					break;
				default:
					strcat(current_line, "UE:   ");
					html_line += "yellow\">UE:&#160;&#160;&#160;";
					//	format.setForeground(Qt::yellow);

			}; // end: switch (message.message_type)
			   //	html_line += "</font>";
			   //	mw->getMenuBar()->textEdit_sr->setCurrentCharFormat(format);

			std::string text(sr_msg.description);

			boost::char_separator <char> sep("\n");
			boost::tokenizer <boost::char_separator <char> > tokens(text, sep);

			bool first_it = true;
			BOOST_FOREACH(const std::string & t, tokens)
					{

						input = t.c_str();

						html_it(input, output);

						if (first_it) {
							first_it = false;

							html_line += output + "</font>";

						} else {
							html_line =
									"<font face=\"Monospace\" color=\"black\">&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160; "
											"&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;"
											"&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;&#160;"
											+ output + "</font>";

							strcpy(current_line, "                                                     ");
						}
						strcat(current_line, t.c_str());

						mw->get_ui()->textEdit_sr->append(QString::fromStdString(html_line));

						(*log_file_outfile) << current_line << std::endl;
					}
		}

		(*log_file_outfile).flush();

	}

	if (ui_state == 2) { // jesli ma nastapic zamkniecie z aplikacji
		set_ui_state_notification(UI_N_EXITING);
		// 	printf("w ontimer 2\n");
		closing_delay_counter = 20; // opoznienie zamykania
		ui_state = 3;
		// 		delay(5000);

		mp->MPslay();

		ui_msg->message("closing");
	} else if (ui_state == 3) { // odliczanie
		// 	printf("w ontimer 3\n");
		if ((--closing_delay_counter) <= 0)
			ui_state = 4;
	} else if (ui_state == 4) { // jesli ma nastapic zamkniecie aplikacji
		//	printf("w ontimer 4\n");
		closing_delay_counter = 20; // opoznienie zamykania
		ui_state = 5;

		all_robots->EDP_all_robots_slay();

	} else if (ui_state == 5) { // odlcizanie do zamnkiecia
		//	printf("w ontimer 5\n");
		if ((--closing_delay_counter) <= 0)
			ui_state = 6;
	} else if (ui_state == 6) { // zakonczenie aplikacji
		(*log_file_outfile).close();
		delete log_file_outfile;
		printf("UI CLOSED\n");
		abort_threads();
		get_main_window()->close();

	} else {
		if (!(communication_flag.is_busy())) {
			set_ui_state_notification(UI_N_READY);
		}

	}

}

void Interface::raise_process_control_window()
{
	//ui->notification_label->setText("GUGUGU");
	emit raise_process_control_window_signal();
}

void Interface::raise_process_control_window_slot()
{
	wgt_pc->my_open(false);
	wgt_pc->raise();
	open_process_control_windows();
	//
}

void Interface::open_process_control_windows()
{
	//	BOOST_FOREACH(wgt_robot_process_control *wgt_robot, wgt_robots_pc)
	//	{
	//		wgt_robot->close();
	//		delete	wgt_robot;
	//	}
	//
	//	wgt_robots_pc.clear();
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				if ((robot_node.second->state.is_active) && (robot_node.second->is_edp_loaded())) {
					robot_node.second->get_wgt_robot_pc()->my_open();
				}
			}

	//	BOOST_FOREACH(wgt_robot_process_control *wgt_robot, wgt_robots_pc)
	//	{
	//		wgt_robot->my_open();
	//
	//	}

}

//Interface * Interface::get_instance()
//{
//	static Interface *instance = new Interface();
//	return instance;
//}

void Interface::raise_ui_ecp_window()
{
	//ui->notification_label->setText("GUGUGU");
	emit raise_ui_ecp_window_signal();
}

void Interface::raise_ui_ecp_window_slot()
{
	ui_msg->message("raise_ui_ecp_window_slot");

	lib::ECP_message &ecp_to_ui_msg = ui_ecp_obj->ecp_to_ui_msg;
	lib::UI_reply &ui_rep = ui_ecp_obj->ui_rep;

	switch (ecp_to_ui_msg.ecp_message)
	{ // rodzaj polecenia z ECP
		case lib::C_XYZ_ANGLE_AXIS: {
			if (teachingstate == ui::common::MP_RUNNING) {
				teachingstate = ui::common::ECP_TEACHING;
			}

			Ui::wgt_teachingClass* ui = wgt_teaching_obj->get_ui();

			ui->label_message->setText("C_XYZ_ANGLE_AXIS");

			wgt_teaching_obj->my_open();

			if (ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6ot_m::ROBOT_NAME) {
				/* TR
				 start_wnd_irp6_on_track_xyz_angle_axis(widget, apinfo, cbinfo);
				 */
			} else if (ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6p_m::ROBOT_NAME) {
				/* TR
				 start_wnd_irp6_postument_xyz_angle_axis(widget, apinfo, cbinfo);
				 */
			}

		}
			break;
		case lib::C_XYZ_EULER_ZYZ: {
			if (teachingstate == ui::common::MP_RUNNING) {
				teachingstate = ui::common::ECP_TEACHING;
			}

			Ui::wgt_teachingClass* ui = wgt_teaching_obj->get_ui();

			ui->label_message->setText("C_XYZ_EULER_ZYZ");

			wgt_teaching_obj->my_open();

			if (ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6ot_m::ROBOT_NAME) {
				/* TR
				 start_wnd_irp6_on_track_xyz_euler_zyz(widget, apinfo, cbinfo);
				 */
			} else if (ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6p_m::ROBOT_NAME) {
				/* TR
				 start_wnd_irp6_postument_xyz_euler_zyz(widget, apinfo, cbinfo);
				 */
			}

		}
			break;
		case lib::C_JOINT: {
			if (teachingstate == ui::common::MP_RUNNING) {
				teachingstate = ui::common::ECP_TEACHING;
			}

			Ui::wgt_teachingClass* ui = wgt_teaching_obj->get_ui();

			ui->label_message->setText("C_JOINT");

			wgt_teaching_obj->my_open();

			if (ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6ot_m::ROBOT_NAME) { //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				robot_m[lib::irp6ot_m::ROBOT_NAME]->wgts[irp6p_m::UiRobot::WGT_JOINTS]->my_open();
			} else if (ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6p_m::ROBOT_NAME) {
				robot_m[lib::irp6p_m::ROBOT_NAME]->wgts[irp6p_m::UiRobot::WGT_JOINTS]->my_open();
			}

		}
			break;
		case lib::C_MOTOR: {
			//  printf("C_MOTOR\n");

			if (teachingstate == ui::common::MP_RUNNING) {
				teachingstate = ui::common::ECP_TEACHING;
			}

			Ui::wgt_teachingClass* ui = wgt_teaching_obj->get_ui();

			ui->label_message->setText("C_MOTOR");

			wgt_teaching_obj->my_open();

			if (ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6ot_m::ROBOT_NAME)
				robot_m[lib::irp6ot_m::ROBOT_NAME]->wgts[irp6p_m::UiRobot::WGT_MOTORS]->my_open();
			else if (ui_ecp_obj->ecp_to_ui_msg.robot_name == lib::irp6p_m::ROBOT_NAME)
				robot_m[lib::irp6p_m::ROBOT_NAME]->wgts[irp6p_m::UiRobot::WGT_MOTORS]->my_open();

		}
			break;
		case lib::YES_NO: {
			Ui::wgt_yes_noClass* ui = wgt_yes_no_obj->get_ui();

			ui->label_message->setText(ecp_to_ui_msg.string);

			wgt_yes_no_obj->my_open();

		}
			break;
		case lib::MESSAGE: {
			Ui::wgt_messageClass* ui = wgt_message_obj->get_ui();
			ui->label_message->setText(ecp_to_ui_msg.string);
			wgt_message_obj->my_open();

			ui_rep.reply = lib::ANSWER_YES;
			ui_ecp_obj->synchroniser.command();
		}
			break;
		case lib::DOUBLE_NUMBER: {

			Ui::wgt_input_doubleClass* ui = wgt_input_double_obj->get_ui();

			ui->label_message->setText(ecp_to_ui_msg.string);

			wgt_input_double_obj->my_open();
		}
			break;
		case lib::INTEGER_NUMBER: {

			Ui::wgt_input_integerClass* ui = wgt_input_integer_obj->get_ui();

			ui->label_message->setText(ecp_to_ui_msg.string);

			wgt_input_integer_obj->my_open();

		}
			break;
		case lib::CHOOSE_OPTION: {

			Ui::wgt_choose_optionClass* ui = wgt_choose_option_obj->get_ui();

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

			wgt_choose_option_obj->my_open();
		}
			break;
		case lib::LOAD_FILE: {
			// Zaladowanie pliku - do ECP przekazywana jest nazwa pliku ze sciezka

			//    printf("lib::LOAD_FILE\n");

			file_window_mode = ui::common::FSTRAJECTORY;

			try {
				QString fileName;

				fileName =
						QFileDialog::getOpenFileName(mw.get(), tr("Choose file to load or die"), mrrocpp_root_local_path.c_str(), tr("Image Files (*)"));

				if (fileName.length() > 0) {

					strncpy(ui_ecp_obj->ui_rep.filename, rindex(fileName.toStdString().c_str(), '/') + 1, strlen(rindex(fileName.toStdString().c_str(), '/'))
							- 1);
					ui_ecp_obj->ui_rep.filename[strlen(rindex(fileName.toStdString().c_str(), '/')) - 1] = '\0';

					strncpy(ui_ecp_obj->ui_rep.path, fileName.toStdString().c_str(), strlen(fileName.toStdString().c_str())
							- strlen(rindex(fileName.toStdString().c_str(), '/')));
					ui_ecp_obj->ui_rep.path[strlen(fileName.toStdString().c_str())
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

			ui_ecp_obj->synchroniser.command();

		}
			break;
		case lib::SAVE_FILE: {

			// Zapisanie do pliku - do ECP przekazywana jest nazwa pliku ze sciezka
			//    printf("lib::SAVE_FILE\n");

			file_window_mode = ui::common::FSTRAJECTORY;

			try {
				QString fileName;

				fileName =
						QFileDialog::getSaveFileName(mw.get(), tr("Choose file to save or die"), mrrocpp_root_local_path.c_str(), tr("Image Files (*)"));

				if (fileName.length() > 0) {

					strncpy(ui_ecp_obj->ui_rep.filename, rindex(fileName.toStdString().c_str(), '/') + 1, strlen(rindex(fileName.toStdString().c_str(), '/'))
							- 1);
					ui_ecp_obj->ui_rep.filename[strlen(rindex(fileName.toStdString().c_str(), '/')) - 1] = '\0';

					strncpy(ui_ecp_obj->ui_rep.path, fileName.toStdString().c_str(), strlen(fileName.toStdString().c_str())
							- strlen(rindex(fileName.toStdString().c_str(), '/')));
					ui_ecp_obj->ui_rep.path[strlen(fileName.toStdString().c_str())
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

			ui_ecp_obj->synchroniser.command();

		}
			break;

		default: {
			perror("Strange ECP message");
			ui_ecp_obj->synchroniser.command();
		}
			break;
	}

}

void Interface::setRobotsMenu()
{
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				robot_node.second->setup_menubar();
			}
}

MainWindow* Interface::get_main_window() const
{
	return mw.get();
}

int Interface::set_ui_state_notification(UI_NOTIFICATION_STATE_ENUM new_notifacion)
{
	{
		boost::unique_lock <boost::mutex> lock(ui_notification_state_mutex);

		next_notification = new_notifacion;
	}

	mw->ui_notification();

	return 1;
}

int Interface::wait_for_child_termiantion(pid_t pid)
{
	int status;
	pid_t child_pid;
	child_pid = waitpid(pid, &status, 0);

	if (child_pid == -1) {
		//	int e = errno;
		perror("UI: waitpid()");
	} else if (child_pid == 0) {
		fprintf(stderr, "UI: no child exited\n");
	} else {
		//fprintf(stderr, "UI: child %d...\n", child_pid);
		if (WIFEXITED(status)) {
			fprintf(stderr, "UI: child %d exited normally with status %d\n", child_pid, WEXITSTATUS(status));
		}
		if (WIFSIGNALED(status)) {
#ifdef WCOREDUMP
			if (WCOREDUMP(status)) {
				fprintf(stderr, "UI: child %d terminated by signal %d (core dumped)\n", child_pid, WTERMSIG(status));
			} else
#endif /* WCOREDUMP */
			{
				fprintf(stderr, "UI: child %d terminated by signal %d\n", child_pid, WTERMSIG(status));
			}
		}
		if (WIFSTOPPED(status)) {
			fprintf(stderr, "UI: child %d stopped\n", child_pid);
		}
		if (WIFCONTINUED(status)) {
			fprintf(stderr, "UI: child %d resumed\n", child_pid);
		}
	}

	return status;
}

common::robots_t Interface::getRobots() const
{
	return robot_m;
}

void Interface::create_robots()
{
	ADD_UI_ROBOT(spkm1);
	ADD_UI_ROBOT(spkm2);
	ADD_UI_ROBOT(smb1);
	ADD_UI_ROBOT(smb2);
	ADD_UI_ROBOT(shead1);
	ADD_UI_ROBOT(shead2);
	ADD_UI_ROBOT(sbench);
	ADD_UI_ROBOT(irp6ot_m);
	ADD_UI_ROBOT(irp6p_m);
	ADD_UI_ROBOT(polycrank);
	ADD_UI_ROBOT(bird_hand);
	ADD_UI_ROBOT(sarkofag);
	ADD_UI_ROBOT(irp6p_tfg);
	ADD_UI_ROBOT(conveyor);
	ADD_UI_ROBOT(irp6ot_tfg);

	setRobotsMenu();
}

void Interface::init()
{

	// dodanie innych okien w dock widgetach
	wgt_pc = new wgt_process_control(*this);
	wgt_yes_no_obj = new wgt_yes_no(*this);
	wgt_message_obj = new wgt_message(*this);
	wgt_input_integer_obj = new wgt_input_integer(*this);
	wgt_input_double_obj = new wgt_input_double(*this);
	wgt_choose_option_obj = new wgt_choose_option(*this);
	wgt_teaching_obj = new wgt_teaching(*this);

	// ustalenie katalogow UI

	mw->show();

	/*
	 dwgt_pc = new QDockWidget(mw);
	 //dwgt_pc->setAllowedAreas(Qt::TopDockWidgetArea);
	 dwgt_pc->setWindowTitle("Process control");

	 vl_pc = new QVBoxLayout();
	 dwgt_pc->setLayout(vl_pc);

	 vl_pc->addWidget(wgt_pc);
	 dwgt_pc->setWidget(wgt_pc);
	 dwgt_pc->hide();
	 mw->addDockWidget(Qt::LeftDockWidgetArea, dwgt_pc);
	 */
	struct utsname sysinfo;
	char* cwd;
	char buff[PATH_MAX + 1];

if(	uname(&sysinfo) == -1) {
		perror("uname");
	}

	cwd = getcwd(buff, PATH_MAX + 1);
	if (cwd == NULL) {
		perror("Blad cwd w UI");
	}

	ui_node_name = sysinfo.nodename;

	binaries_local_path = cwd;
	mrrocpp_local_path = cwd;
	mrrocpp_local_path.erase(mrrocpp_local_path.length() - 3); // kopiowanie lokalnej sciezki bez "bin" - 3 znaki
	//mrrocpp_local_path += "../";
	mrrocpp_root_local_path = mrrocpp_local_path.substr(0, mrrocpp_local_path.rfind("/"));
	mrrocpp_root_local_path = mrrocpp_root_local_path.substr(0, mrrocpp_root_local_path.rfind("/") + 1);
	binaries_network_path = "/net/";
	binaries_network_path += ui_node_name;
	binaries_network_path += binaries_local_path;
	binaries_network_path += "/"; // wysylane jako argument do procesow potomnych (mp_m i dalej)
	// printf( "system name  : %s\n", binaries_network_path);

	// sciezka dla okna z wyborem pliku podczas wybor trajektorii dla uczenia
	teach_filesel_fullpath = "/net/";
	teach_filesel_fullpath += ui_node_name;
	teach_filesel_fullpath += mrrocpp_local_path;
	//	teach_filesel_fullpath += "trj";
	// 	printf("abba: %s\n", teach_filesel_fullpath);

	// sciezka dla okna z wyborem pliku z trajektoria podczas wyboru pliku konfiguracyjnego
	config_file_fullpath = "/net/";
	config_file_fullpath += ui_node_name;
	config_file_fullpath += mrrocpp_local_path;
	config_file_fullpath += "../";

	// printf ("Remember to create gns server\n");

	set_ui_state_notification(UI_N_STARTING);

	signal(SIGINT, &catch_signal); // by y aby uniemozliwic niekontrolowane zakonczenie aplikacji ctrl-c z kalwiatury
	signal(SIGALRM, &catch_signal);
	signal(SIGSEGV, &catch_signal);

	// signal(SIGCHLD, &catch_signal);
	/* TR
	 lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 6);
	 */
	// pierwsze zczytanie pliku konfiguracyjnego (aby pobrac nazwy dla pozostalych watkow UI)
	if (get_default_configuration_file_name() >= 1) // zczytaj nazwe pliku konfiguracyjnego
			{

		initiate_configuration();
		// sprawdza czy sa postawione gns's i ew. stawia je
		// uwaga serwer musi byc wczesniej postawiony
		check_gns();
	} else {
		printf("Blad manage_default_configuration_file\n");
		/* TR
		 PtExit(EXIT_SUCCESS);
		 */
	}

	create_threads();

	create_robots();

	// Zablokowanie domyslnej obslugi sygnalu SIGINT w watkach UI_SR i UI_COMM

	// kolejne zczytanie pliku konfiguracyjnego
	if (get_default_configuration_file_name() == 1) // zczytaj nazwe pliku konfiguracyjnego
			{

		reload_whole_configuration();

	} else {
		printf("Blad manage_default_configuration_file\n");
		/* TR
		 PtExit(EXIT_SUCCESS);
		 */
	}

	// inicjacja pliku z logami sr
	check_gns();

	time_t time_of_day;
	char file_date[50];
	char log_file_with_dir[100];
	char file_name[50];

	time_of_day = time(NULL);
	strftime(file_date, 40, "%g%m%d_%H-%M-%S", localtime(&time_of_day));

	sprintf(file_name, "/%s_sr_log", file_date);

	// 	strcpy(file_name,"/pomiar.p");
	strcpy(log_file_with_dir, (mrrocpp_bin_to_root_path + "logs/").c_str());

	if (access(log_file_with_dir, R_OK) != 0) {
		mkdir(log_file_with_dir, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
	}

	strcat(log_file_with_dir, file_name);

	// C++ new does not return 0 on failure, so there is no need to check
	log_file_outfile = new std::ofstream(log_file_with_dir, std::ios::out);

	//ui_msg->message("closing");

	//manage_interface();

	mw->get_ui()->textEdit_sr->setFocus();

}

//void Interface::print_on_sr(const std::string &text)
void Interface::print_on_sr(const char *buff, ...)
{
	char text[256];
	va_list arglist;

	va_start(arglist,buff);
	vsprintf(text, buff, arglist);
	va_end(arglist);

	ui_msg->message(text);
}

void Interface::manage_pc(void)
{
	wgt_pc->process_control_window_init();

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				if ((robot_node.second->state.is_active) && (robot_node.second->is_edp_loaded())) {
					robot_node.second->get_wgt_robot_pc()->process_control_window_init();
				}
			}

	//wgt_pc->dwgt->raise();
	//	BOOST_FOREACH(wgt_robot_process_control *wgt_robot, wgt_robots_pc)
	//	{
	//		wgt_robot->process_control_window_init();
	//	}

}

// funkcja odpowiedzialna za wyglad aplikacji na podstawie jej stanu

int Interface::manage_interface(void)
{
	emit
	manage_interface_signal();

	return 1;
}

void Interface::manage_interface_slot()
{
	// okienko process control
	wgt_pc->process_control_window_init_slot();

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				if ((robot_node.second->state.is_active) && (robot_node.second->is_edp_loaded())
						&& robot_node.second->get_wgt_robot_pc()) {
					robot_node.second->get_wgt_robot_pc()->process_control_window_init();
				}
			}

	//wgt_pc->dwgt->raise();
	// UWAGA ta funkcja powinna byc odporna na odpalenie z dowolnego watku !!!

	check_edps_state_and_modify_mp_state();
	/*TR
	 // na wstepie wylaczamy przyciski EDP z all robots menu. Sa one ewentualnie wlaczane dalej
	 ApModifyItemState(&all_robots_menu, AB_ITEM_DIM, ABN_mm_all_robots_preset_positions,
	 ABN_mm_all_robots_synchronisation, ABN_mm_all_robots_edp_unload, ABN_mm_all_robots_edp_load, NULL);

	 // menu file
	 // ApModifyItemState( &file_menu, AB_ITEM_DIM, NULL);

	 */
	// uruchmomienie manage interface dla wszystkich robotow
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				robot_node.second->manage_interface();
			}

	// wlasciwosci menu  ABW_base_all_robots

	all_robots->manage_interface();
	mp->manage_interface();

}

void Interface::reload_whole_configuration()
{

	if (access(config_file_relativepath.c_str(), R_OK) != 0) {
		std::cerr << "Wrong entry in default_file.cfg - load another configuration than: " << config_file_relativepath
				<< std::endl;
		config_file_relativepath = mrrocpp_bin_to_root_path + "configs/common.ini";
	}

	if ((mp->mp_state.state == UI_MP_NOT_PERMITED_TO_RUN) || (mp->mp_state.state == UI_MP_PERMITED_TO_RUN)) { // jesli nie dziala mp podmien mp ecp vsp

		config->change_config_file("../" + config_file);

		is_mp_and_ecps_active = config->exists_and_true("is_active", "[mp]");

		switch (all_robots->all_edps)
		{
			case UI_ALL_EDPS_NONE_ACTIVATED:
			case UI_ALL_EDPS_NONE_LOADED:

				// uruchmomienie manage interface dla wszystkich robotow
				BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
						{
							robot_node.second->reload_configuration();
						}

				break;
			default:
				break;
		}

		// clearing of lists
		clear_all_configuration_lists();

		// sczytanie listy sekcji
		fill_section_list(config_file_relativepath.c_str());
		fill_section_list((mrrocpp_bin_to_root_path + "configs/common.ini").c_str());
		fill_node_list();
		fill_program_node_list();

		/*
		 for (list<char*>::iterator list_iterator = section_list.begin(); list_iterator != section_list.end(); list_iterator++)
		 {
		 printf("section_name: %s\n", *list_iterator);

		 }

		 for (list<char*>::iterator node_list_iterator = ui_state.node_list.begin(); node_list_iterator != ui_state.node_list.end(); node_list_iterator++)
		 {
		 printf("node_name: %s\n", *node_list_iterator);
		 }

		 for (list<program_node_def>::iterator program_node_list_iterator = program_node_list.begin(); program_node_list_iterator != program_node_list.end(); program_node_list_iterator++)
		 {
		 printf("node_name: %s\n", program_node_list_iterator->node_name);
		 }
		 */

		// zczytanie konfiguracji UI

		// zczytanie konfiguracji MP
		if (is_mp_and_ecps_active) {
			mp->mp_state.network_pulse_attach_point = config->get_mp_pulse_attach_point();

			if (!config->exists("node_name", lib::MP_SECTION)) {
				mp->mp_state.node_name = "localhost";
			} else {
				mp->mp_state.node_name = config->value <std::string>("node_name", lib::MP_SECTION);
			}

			mp->mp_state.pid = -1;
		}

		// inicjacja komunikacji z watkiem sr
		if (ui_msg == NULL) {
			ui_msg =
					(boost::shared_ptr <lib::sr_ui>) new lib::sr_ui(lib::UI, ui_attach_point.c_str(), network_sr_attach_point);
		}

		// wypisanie komunikatu o odczytaniu konfiguracji
		if (ui_msg) {
			std::string msg(config_file);
			msg += " config file loaded";
			ui_msg->message(msg.c_str());
		}
		mw->get_ui()->label_config_file_notification->setText(config_file.c_str());

	}

	manage_interface();
}

void Interface::UI_close(void)
{
	if (ui_state < 2) {
		printf("UI CLOSING\n");

		// czas na ustabilizowanie sie edp
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));

		ui_state = 2; // funcja OnTimer dowie sie ze aplikacja ma byc zamknieta
	}
}

void Interface::abort_threads()
{
	// Note: these originally were a pointers to a threaded objects,
	// and they were deleted here.
	ui_sr_obj.reset();
	ui_ecp_obj.reset();
	meb_tid.reset();
}

bool Interface::check_node_existence(const std::string & _node, const std::string & beginnig_of_message)
{
	bool r_val;

	{
		boost::unique_lock <boost::mutex> lock(process_creation_mtx);
		r_val = lib::ping(_node);
	}

	std::cout << "ping returned " << r_val << std::endl;

	if (!r_val) {

		std::string tmp(beginnig_of_message);
		tmp += std::string(" node: ") + _node + std::string(" is unreachable");
		ui_msg->message(lib::NON_FATAL_ERROR, tmp);

		return false;
	}

	return true;

}

// sprawdza czy sa postawione gns's i ew. stawia je
// uwaga serwer powinien byc wczesniej postawiony (dokladnie jeden w sieci)
// jesli dziala messip (np. pod linux) nieaktywne

int Interface::check_gns()
{
	return 1;
}

// ustala stan wszytkich EDP
int Interface::check_edps_state_and_modify_mp_state()
{
	all_robots->set_edp_state();
	// modyfikacja stanu MP przez stan wszystkich EDP

	mp->set_mp_state();

	return 1;
}

// odczytuje nazwe domyslengo pliku konfiguracyjnego, w razie braku ustawia common.ini
int Interface::get_default_configuration_file_name()
{

	FILE * fp = fopen((mrrocpp_bin_to_root_path + "configs/default_file.cfg").c_str(), "r");
	if (fp != NULL) {
		//printf("alala\n");
		char tmp_buf[255];
		fgets(tmp_buf, 255, fp); // Uwaga na zwracanego NULLa
		char *tmp_buf1 = strtok(tmp_buf, "=\n\r"); // get first token
		config_file = tmp_buf1;

		config_file_relativepath = mrrocpp_bin_to_root_path;
		config_file_relativepath += config_file;

		fclose(fp);
		return 1;

	} else {
		//	printf("balala\n");
		// jesli plik z domyslna konfiguracja (default_file.cfg) nie istnieje to utworz go i wpisz do niego common.ini
		printf("Utworzono plik default_file.cfg z konfiguracja common.ini\n");
		fp = fopen((mrrocpp_bin_to_root_path + "configs/default_file.cfg").c_str(), "w");
		fclose(fp);

		config_file = "configs/common.ini";
		config_file_relativepath = mrrocpp_bin_to_root_path;
		config_file_relativepath += config_file;

		std::ofstream outfile((mrrocpp_bin_to_root_path + "configs/default_file.cfg").c_str(), std::ios::out);
		if (!outfile.good()) {
			std::cerr << "Cannot open file: default_file.cfg" << std::endl;
			perror("because of");
		} else {
			outfile << config_file;
		}
		return 2;
	}
}

// zapisuje nazwe domyslengo pliku konfiguracyjnego
int Interface::set_default_configuration_file_name()
{

	config_file_relativepath = mrrocpp_bin_to_root_path;
	config_file_relativepath += config_file;

	std::ofstream outfile((mrrocpp_bin_to_root_path + "configs/default_file.cfg").c_str(), std::ios::out);
	if (!outfile.good()) {
		std::cerr << "Cannot open file: default_file.cfg\n";
		perror("because of");
	} else
		outfile << config_file;

	return 1;
}

// fills program_node list
int Interface::fill_program_node_list()
{
	//	printf("fill_program_node_list\n");

	for (std::list <Interface::list_t>::iterator section_list_iterator = section_list.begin();
			section_list_iterator != section_list.end(); section_list_iterator++) {

		if (config->exists("program_name", *section_list_iterator)
				&& config->exists("is_active", *section_list_iterator)
				&& config->value <bool>("is_active", *section_list_iterator)) {
			//	char* tmp_p =config->value<std::string>("program_name", *section_list_iterator);
			//	char* tmp_n =config->value<std::string>("node_name", *section_list_iterator);

			program_node_user_def tmp_s;

			tmp_s.program_name = config->value <std::string>("program_name", *section_list_iterator);
			if (config->exists("node_name", *section_list_iterator)) {
				tmp_s.node_name = config->value <std::string>("node_name", *section_list_iterator);
			} else {
				tmp_s.node_name = std::string("localhost");
			}

			if (config->exists("username", *section_list_iterator)) {
				tmp_s.user_name = config->value <std::string>("username", *section_list_iterator);
			} else {
				tmp_s.user_name = getenv("USER");
			}

			if (config->exists("is_qnx", *section_list_iterator)) {
				tmp_s.is_qnx = config->value <bool>("is_qnx", *section_list_iterator);
			} else {
				tmp_s.is_qnx = false;
			}

			program_node_user_list.push_back(tmp_s);
		}
	}

	return 1;
}

int Interface::clear_all_configuration_lists()
{
	// clearing of lists
	section_list.clear();
	config_node_list.clear();
	all_node_list.clear();
	program_node_user_list.clear();

	return 1;
}

int Interface::initiate_configuration()
{

	if (access(config_file_relativepath.c_str(), R_OK) != 0) {
		fprintf(stderr, "Wrong entry in default_file.cfg - load another configuration than: %s\n", config_file_relativepath.c_str());
		config_file_relativepath = mrrocpp_bin_to_root_path + "configs/common.ini";
	}

	// sprawdzenie czy nazwa sesji jest unikalna

	bool wyjscie = false;

	while (!wyjscie) {

		config.reset();

		config =
				(boost::shared_ptr <lib::configurator>) new lib::configurator(ui_node_name, mrrocpp_local_path, lib::UI_SECTION);

		std::string attach_point = config->get_sr_attach_point();

		// wykrycie identycznych nazw sesji
		wyjscie = true;

		DIR* dirp = opendir("/dev/name/global");

		if (dirp != NULL) {
			for (;;) {
				struct dirent* direntp = readdir(dirp);
				if (direntp == NULL
					)
					break;

				// printf( "%s\n", direntp->d_name );
				if (attach_point == direntp->d_name) {
					wyjscie = false;
				}
			}

			closedir(dirp);
		}

	}

	ui_attach_point = config->get_ui_attach_point();
	sr_attach_point = config->get_sr_attach_point();
	network_sr_attach_point = config->get_sr_attach_point();

	clear_all_configuration_lists();

	// sczytanie listy sekcji
	fill_section_list(config_file_relativepath.c_str());
	fill_section_list((mrrocpp_bin_to_root_path + "configs/common.ini").c_str());
	fill_node_list();
	fill_program_node_list();

	return 1;
}

// fills section list of configuration files
int Interface::fill_section_list(const char* file_name_and_path)
{
	static char line[256];

	// otworz plik konfiguracyjny
	FILE * file = fopen(file_name_and_path, "r");
	if (file == NULL) {
		printf("UI fill_section_list Wrong file_name: %s\n", file_name_and_path);
		/* TR
		 PtExit(EXIT_SUCCESS);
		 */
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

			std::list <Interface::list_t>::iterator list_iterator;

			// checking if section is already considered
			for (list_iterator = section_list.begin(); list_iterator != section_list.end(); list_iterator++) {
				if ((*list_iterator) == current_section)
					break;
			}

			// if the section does not exists
			if (list_iterator == section_list.end()) {
				section_list.push_back(std::string(current_section));
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
void Interface::fill_node_list()
{
	// fill all network nodes list

	DIR* dirp = opendir("/net");
	if (dirp != NULL) {
		for (;;) {
			struct dirent *direntp = readdir(dirp);
			if (direntp == NULL
				)
				break;
			all_node_list.push_back(std::string(direntp->d_name));
		}
		closedir(dirp);
	}

	for (std::list <Interface::list_t>::iterator section_list_iterator = section_list.begin();
			section_list_iterator != section_list.end(); section_list_iterator++) {
		if (config->exists("node_name", *section_list_iterator)) {
			std::string tmp = config->value <std::string>("node_name", *section_list_iterator);

			std::list <Interface::list_t>::iterator node_list_iterator;

			for (node_list_iterator = config_node_list.begin(); node_list_iterator != config_node_list.end();
					node_list_iterator++) {
				if (tmp == (*node_list_iterator)) {
					break;
				}
			}

			// if the node does not exists
			if (node_list_iterator == config_node_list.end()) {
				config_node_list.push_back(tmp);
			}
		}

	}
}

void Interface::create_threads()
{
	ui_sr_obj = (boost::shared_ptr <sr_buffer>) new sr_buffer(*this);

	meb_tid = (boost::shared_ptr <feb_thread>) new feb_thread(*main_eb);

	ui_sr_obj->thread_started.wait();

	ui_ecp_obj = (boost::shared_ptr <ecp_buffer>) new ecp_buffer(*this);

	start_on_timer();
}

// zatrzymuje zadanie, zabija procesy
int Interface::unload_all()
{
	mp->MPslay();

	boost::this_thread::sleep(boost::posix_time::milliseconds(200));

	all_robots->EDP_all_robots_slay();
	/* TR
	 close_process_control_window(widget, apinfo, cbinfo);
	 */
	return 1;

}

// najpierw unload_all zabija wszystkie procesy wzmiankowane w pliku konfiguracyjnym

int Interface::slay_all()
{

	// program unload

	unload_all();

	// brutal overkilling

	for (std::list <ui::common::program_node_user_def>::iterator program_node_user_list_iterator =
			program_node_user_list.begin(); program_node_user_list_iterator != program_node_user_list.end();
			program_node_user_list_iterator++) {
		char system_command[100];
		/*
		 #if 0
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
		 */delay(100);

		sprintf(system_command, "rsh -l %s %s killall -9 -e -q %s", program_node_user_list_iterator->user_name.c_str(), program_node_user_list_iterator->node_name.c_str(), program_node_user_list_iterator->program_name.c_str());

		printf("slay_all: %s\n", system_command);
		// przedniolsem wywolanie system do innego prceosu bo w procesie glownym czasem powoduje zawieszenie calego intefrejsu

		system(system_command);

	}
	printf("slay_all end\n");
	manage_interface();

	return 1;

}

}
}
}
