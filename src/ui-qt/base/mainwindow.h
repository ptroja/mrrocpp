#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QCloseEvent>
#include <QSignalMapper>
#include <pthread.h>
#include "wgt_base.h"
#include "ui_robot.h"
#include "signal_dispatcher.h"
#include "menu_bar.h"

namespace Ui {
class MainWindow;
class SignalDispatcher;
}

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
//class UiRobot;
}
}
}

namespace mrrocpp {
namespace ui {
namespace irp6ot_m {
class UiRobot;
}
}
}

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
	explicit MainWindow(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~MainWindow();

	void ui_notification();

	void open_new_window(wgt_base *window, wgt_base::my_open_ptr func, bool set_on_top = false);
	void open_new_window(wgt_base *window, bool set_on_top = false);

	void ui_robot_action(mrrocpp::ui::common::UiRobot* robot, mrrocpp::ui::common::UiRobot::uiRobotFunctionPointer pointer);
	void ui_robot_action(mrrocpp::ui::common::UiRobot* robot, mrrocpp::ui::common::UiRobot::intUiRobotFunctionPointer pointer);
	void ui_robot_action(mrrocpp::ui::common::UiRobot* robot, mrrocpp::ui::common::UiRobot::intUiRobotFunctionPointerInt pointer, int argument);
	void ui_robot_action(mrrocpp::ui::common::UiRobot* robot, mrrocpp::ui::common::UiRobot::uiRobotFunctionPointerInt pointer, int argument);

	void get_lineEdit_position(double* val, int number_of_servos);

	Ui::MainWindow * get_ui();

	void closeEvent(QCloseEvent * event);

	wgt_base::my_open_ptr getFunctionPointer();
	Ui::SignalDispatcher* getSignalDispatcher();

	Ui::MenuBar* getMenuBar();

	mrrocpp::ui::common::Interface* getInterface();
	void setMenu();
	void clear_console();

private:
	Ui::MainWindow *ui;
	Ui::MenuBar *menuBar;
	mrrocpp::ui::common::Interface& interface;
	Ui::SignalDispatcher *signalDispatcher;

	pthread_t main_thread_id;

	wgt_base::my_open_ptr openFunctionPointer;

	mrrocpp::ui::common::UiRobot::uiRobotFunctionPointer uiRobotFunctionPtr;
	mrrocpp::ui::common::UiRobot::uiRobotFunctionPointerInt uiRobotFunctionPtrInt;
	mrrocpp::ui::common::UiRobot::intUiRobotFunctionPointer intUiRobotFunctionPtr;
	mrrocpp::ui::common::UiRobot::intUiRobotFunctionPointerInt intUiRobotFunctionPtrInt;

	// void (mrrocpp::ui::irp6ot_m::UiRobot::*intUiRobotIrp6FunctionPointer)();

signals:
	void ui_notification_signal();
	void clear_console_signal();

//	void open_new_window_signal(wgt_base *window, wgt_base::my_open_ptr func);
	void open_new_window_signal(wgt_base *window, bool set_on_top);

	void ui_robot_signal(mrrocpp::ui::common::UiRobot *robot);
	void ui_robot_signal_int(mrrocpp::ui::common::UiRobot *robot, int argument);
	void ui_robot_int_signal_int(mrrocpp::ui::common::UiRobot *robot, int argument);
	void ui_robot_int_signal(mrrocpp::ui::common::UiRobot *robot);

	void raise_process_control_window_signal();
	void raise_ui_ecp_window_signal();

private slots:
	void ui_notification_slot();

	void clear_console_slot();

	//void open_new_window_slot(wgt_base *window, wgt_base::my_open_ptr func);
	void open_new_window_slot(wgt_base *window, bool set_on_top);

	void ui_robot_slot(mrrocpp::ui::common::UiRobot *robot);
	void ui_robot_slot_int(mrrocpp::ui::common::UiRobot *robot, int argument);
	void ui_robot_int_slot_int(mrrocpp::ui::common::UiRobot *robot, int argument);
	void ui_robot_int_slot(mrrocpp::ui::common::UiRobot *robot);

};

#endif // MAINWINDOW_H
