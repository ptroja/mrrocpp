#include <QTextCharFormat>
#include <QBrush>
#include <QColor>
#include <QFileDialog>
#include <QThread>

#include <ctime>
#include <fstream>

#include "mainwindow.h"
#include "ui_mainwindow.h"
//#include "signal_dispatcher.h"

#include "ui.h"
#include "interface.h"
#include "ui_sr.h"
#include "ui_ecp.h"

////#include "../irp6ot_m/ui_r_irp6ot_m.h"
//#include "../irp6p_m/ui_r_irp6p_m.h"
//#include "../irp6p_tfg/ui_r_irp6p_tfg.h"
//#include "../irp6ot_tfg/ui_r_irp6ot_tfg.h"
//#include "../spkm/ui_r_spkm.h"
//#include "../smb/ui_r_smb.h"
//#include "../shead/ui_r_shead.h"
//#include "../polycrank/ui_r_polycrank.h"
//#include "../bird_hand/ui_r_bird_hand.h"
//#include "../sarkofag/ui_r_sarkofag.h"
//#include "../conveyor/ui_r_conveyor.h"
//
//#include "../spkm/wgt_spkm_inc.h"
//#include "../spkm/wgt_spkm_int.h"
//#include "../spkm/wgt_spkm_ext.h"
//#include "../polycrank/wgt_polycrank_int.h"
//#include "../base/wgt_single_motor_move.h"
//
//#include "../bird_hand/wgt_bird_hand_command.h"
//
//#include "../irp6_m/wgt_irp6_m_joints.h"
//#include "../irp6_m/wgt_irp6_m_motors.h"
//#include "../irp6_m/wgt_irp6_m_euler.h"
//#include "../irp6_m/wgt_irp6_m_angle_axis.h"
//#include "../irp6_m/wgt_irp6_m_relative_angle_axis.h"
//#include "../irp6_m/wgt_irp6_m_tool_angle_axis.h"
//#include "../irp6_m/wgt_irp6_m_tool_euler.h"



#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>





MainWindow::MainWindow(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	QMainWindow(parent), ui(new Ui::MainWindow), interface(_interface)
{
	ui->setupUi(this);

	connect(this, SIGNAL(ui_notification_signal()), this, SLOT(ui_notification_slot()), Qt::QueuedConnection);
	connect(this, SIGNAL(enable_menu_item_signal(QMenu *, bool)), this, SLOT(enable_menu_item_slot(QMenu *, bool)), Qt::QueuedConnection);
	connect(this, SIGNAL(enable_menu_item_signal(QAction *, bool)), this, SLOT(enable_menu_item_slot(QAction *, bool)), Qt::QueuedConnection);
	//connect(this, SIGNAL(open_new_window_signal(wgt_base *, wgt_base::my_open_ptr)), this, SLOT(open_new_window_slot(wgt_base *, wgt_base::my_open_ptr)), Qt::QueuedConnection);
	connect(this, SIGNAL(open_new_window_signal(wgt_base *, bool)), this, SLOT(open_new_window_slot(wgt_base *, bool)), Qt::QueuedConnection);
	//connect(this, SIGNAL(ui_robot_signal(mrrocpp::ui::common::UiRobot *)), this, SLOT(ui_robot_slot(mrrocpp::ui::common::UiRobot *)), Qt::QueuedConnection);

	//connect(this, SIGNAL(ui_robot_signal(mrrocpp::ui::irp6ot_m::UiRobot *)), this, SLOT(ui_robot_slot(mrrocpp::ui::irp6ot_m::UiRobot *)), Qt::QueuedConnection);
	connect(this, SIGNAL(ui_robot_signal(mrrocpp::ui::common::UiRobot *)), this, SLOT(ui_robot_slot(mrrocpp::ui::common::UiRobot *)), Qt::QueuedConnection);
	connect(this, SIGNAL(ui_robot_signal_int(mrrocpp::ui::common::UiRobot *, int)), this, SLOT(ui_robot_slot_int(mrrocpp::ui::common::UiRobot *, int)), Qt::QueuedConnection);
	connect(this, SIGNAL(ui_robot_int_signal_int(mrrocpp::ui::common::UiRobot *, int)), this, SLOT(ui_robot_int_slot_int(mrrocpp::ui::common::UiRobot *, int)), Qt::QueuedConnection);
	connect(this, SIGNAL(ui_robot_int_signal(mrrocpp::ui::common::UiRobot *)), this, SLOT(ui_robot_int_slot(mrrocpp::ui::common::UiRobot *)), Qt::QueuedConnection);
	connect(this, SIGNAL(clear_console_signal()), this, SLOT(clear_console_slot()), Qt::QueuedConnection);



	menuBar = new Ui::MenuBar(&interface, this);
	menuBar->setupMenuBar(this);

	signalDispatcher = new Ui::SignalDispatcher(interface);
	//signalDispatcher = new mrrocpp::ui::common::SignalDispatcher();

	//robotsSignalMapper = new QSignalMapper();


	//open_new_window_signal(wgt_base *window);
	main_thread_id = pthread_self();

	//while(menuBar->actionirp6ot_m_EDP_Load==NULL)
	//{printf("wait");}



}

void MainWindow::setMenu()
{
	menuBar->setupMenuBar(this);
}



Ui::SignalDispatcher* MainWindow::getSignalDispatcher()
{
	return signalDispatcher;
}

mrrocpp::ui::common::Interface* MainWindow::getInterface()
{
	return &interface;
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
	delete signalDispatcher;
}

Ui::MenuBar* MainWindow::getMenuBar()
{
	return menuBar;
}

Ui::MainWindow * MainWindow::get_ui()
{
	return ui;
}

void MainWindow::clear_console()
{
	emit clear_console_signal();
}

void MainWindow::clear_console_slot()
{
	get_ui()->textEdit_sr->clear();
}


void MainWindow::enable_menu_item(bool _enable, int _num_of_menus, QMenu *_menu_item, ...)
{
	va_list menu_items;
	// usuniete bo metoda wolana z dobrego watku przez manage interface_slot

//	 emit
//	 enable_menu_item_signal(_menu_item, _enable);
	if(_menu_item)
	enable_menu_item_slot(_menu_item, _enable);
	va_start(menu_items, _menu_item);

	for (int i = 1; i < _num_of_menus; i++) {
		//interface.print_on_sr("signal");
		// usuniete bo metoda wolana z dobrego watku przez manage interface_slot

//		 emit enable_menu_item_signal(va_arg(menu_items, QMenu *), _enable);
		QMenu *action = va_arg(menu_items, QMenu *);
		if(action)
		enable_menu_item_slot(action, _enable);
	}

	va_end(menu_items);
}

void MainWindow::enable_menu_item(bool _enable, int _num_of_menus, QAction *_menu_item, ...)
{
	va_list menu_items;
	// usuniete bo metoda wolana z dobrego watku przez manage interface_slot

//	 emit
//	 enable_menu_item_signal(_menu_item, _enable);

	if(_menu_item)
	enable_menu_item_slot(_menu_item, _enable);

	va_start(menu_items, _menu_item);

	for (int i = 1; i < _num_of_menus; i++) {
		//interface.print_on_sr("signal");
		// usuniete bo metoda wolana z dobrego watku przez manage interface_slot

//		 emit enable_menu_item_signal(va_arg(menu_items, QAction *), _enable);
		QAction *action = va_arg(menu_items, QAction *);
		if(action)
		enable_menu_item_slot(action, _enable);
	}

	va_end(menu_items);
}

void MainWindow::ui_robot_action(mrrocpp::ui::common::UiRobot * robot, mrrocpp::ui::common::UiRobot::uiRobotFunctionPointer pointer)
{
	//printf("ui robot action void");
	uiRobotFunctionPtr = pointer;
	emit ui_robot_signal(robot);

}

void MainWindow::ui_robot_action(mrrocpp::ui::common::UiRobot * robot, mrrocpp::ui::common::UiRobot::intUiRobotFunctionPointerInt pointer, int argument)
{
	//interface.print_on_sr("ui robot action void arg int");
	intUiRobotFunctionPtrInt = pointer;
	emit ui_robot_int_signal_int(robot, argument);
}

void MainWindow::ui_robot_action(mrrocpp::ui::common::UiRobot * robot, mrrocpp::ui::common::UiRobot::intUiRobotFunctionPointer pointer)
{
	//interface.print_on_sr("ui robot action int");
	intUiRobotFunctionPtr = pointer;
	emit ui_robot_int_signal(robot);
}

void MainWindow::ui_robot_action(mrrocpp::ui::common::UiRobot * robot, mrrocpp::ui::common::UiRobot::uiRobotFunctionPointerInt pointer, int argument)
{
	//interface.print_on_sr("ui robot action int");
	uiRobotFunctionPtrInt = pointer;
	emit ui_robot_signal_int(robot, argument);
}




void MainWindow::ui_robot_slot(mrrocpp::ui::common::UiRobot *robot)
{
	//interface.print_on_sr("ui robot slot void");
	(*robot.*uiRobotFunctionPtr)();
}

void MainWindow::ui_robot_slot_int(mrrocpp::ui::common::UiRobot *robot, int argument)
{
	//interface.print_on_sr("ui robot int slot int\n");
	(*robot.*uiRobotFunctionPtrInt)(argument);
}

void MainWindow::ui_robot_int_slot_int(mrrocpp::ui::common::UiRobot *robot, int argument)
{
	//interface.print_on_sr("ui robot slot void");
	(*robot.*intUiRobotFunctionPtrInt)(argument);
}

void MainWindow::ui_robot_int_slot(mrrocpp::ui::common::UiRobot *robot)
{
	//interface.print_on_sr("ui robot int slot int\n");
	(*robot.*intUiRobotFunctionPtr)();
}


void MainWindow::open_new_window(wgt_base *window, wgt_base::my_open_ptr func, bool set_on_top)
{
	openFunctionPointer = func;
	emit open_new_window_signal(window, set_on_top);
	//open_new_window_slot(window, func);
	//interface.print_on_sr("emit 1");
}

//void MainWindow::open_new_window_slot(wgt_base *window, wgt_base::my_open_ptr func)
//{
//	//interface.print_on_sr("slot 1");
//	(*window.*func)();
//}


void MainWindow::open_new_window(wgt_base *window, bool set_on_top)
{
	emit open_new_window_signal(window, set_on_top);
	//open_new_window_slot(window, func);
	//interface.print_on_sr("emit 2");
}

//void MainWindow::open_new_window(wgt_irp6_m_motors *window)
//{
//	emit open_new_window_signal(window);
//	//open_new_window_slot(window, func);
//	interface.print_on_sr("emit 2222");
//}

void MainWindow::open_new_window_slot(wgt_base *window, bool set_on_top)
{
	//interface.print_on_sr("slot 2");

	//wgt_base::my_open_ptr func2=;

	(*window.*openFunctionPointer)(set_on_top);
	//window->my_open();
}



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

		if (j == number_of_servos)
		{
			break;
		}
		j++;
	}

}

void MainWindow::enable_menu_item_slot(QMenu *_menu_item, bool _active)
{
	//interface.print_on_sr("menu coloring slot");
	//_menu_item->setDisabled(!_active);
	if(_menu_item!=NULL)
//	_menu_item->menuAction()->setVisible(_active); //setDisabled(!_active);//setVisible(_active);
	_menu_item->menuAction()->setDisabled(!_active);

//	if(_menu_item!=NULL && _active && !_menu_item->menuAction()->isVisible())
//		_menu_item->menuAction()->setVisible(true); //setDisabled(!_active);//
//	else if(_menu_item!=NULL && !_active && _menu_item->menuAction()->isVisible())
//		_menu_item->menuAction()->setVisible(false);
//	//if(!_active) _menu_item->hide();

}

void MainWindow::enable_menu_item_slot(QAction *_menu_item, bool _active)
{
	//interface.print_on_sr("menu coloring slot");

//	if(_menu_item!=NULL && _active && !_menu_item->isVisible())
//		_menu_item->setVisible(true); //setDisabled(!_active);//
//	else if(_menu_item!=NULL && !_active && _menu_item->isVisible())
//		_menu_item->setVisible(false);
	if(_menu_item!=NULL)
	_menu_item->setDisabled(!_active);//
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




