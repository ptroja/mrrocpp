#ifndef MENUBAR_H_
#define MENUBAR_H_

#include <QObject>
#include <QMenu>
#include <QAction>
#include <QMenuBar>
#include <QMainWindow>
#include <QSignalMapper>
//#include "mainwindow.h"

namespace mrrocpp {
namespace ui {

namespace common {
class Interface;
class AllRobots;
class Mp;
}
}
}

namespace Ui {
//class MainWindow;
class MenuBarAction;
}

namespace Ui {

class MenuBar : public QMenuBar
{
Q_OBJECT

public:
	MenuBar(mrrocpp::ui::common::Interface* iface, QWidget *parent);

	void setupMenuBar(QMainWindow *mainWindow);

	void makeConnections();

	QAction *actionQuit;
	QAction *actionClear_Console;
	QAction *actionEDP_load_smb;
	QAction *actionUnload_All;
	QAction *actionSlay_All;
	QAction *actionAbout;
	QAction *actionall_EDP_Load;
	QAction *actionall_EDP_Unload;
	QAction *actionall_Synchronisation;
	QAction *actionall_Synchro_Position;
	QAction *actionall_Front_Position;
	QAction *actionall_Position_0;
	QAction *actionall_Position_1;
	QAction *actionall_Position_2;
	QAction *actionMP_Load;
	QAction *actionMP_Unload;
	QAction *actionProcess_Control;
	QAction *actionOpen_Configuration;
	QAction *actionReload_Configuration;
//
//    QAction *actionspeaker_EDP_Load;
//    QAction *actionspeaker_EDP_Unload;
//    QAction *actionspeaker_Play;
//    QAction *actionspeaker_Sound_0;
//    QAction *actionspeaker_Sound_1;
//    QAction *actionspeaker_Sound_2;
//    QAction *actionirp6_Mechatronika_EDP_Load;
//    QAction *actionirp6_Mechatronika_EDP_Unload;
//    QAction *actionirp6_Mechatronika_Synchronization;
//    QAction *actionirp6_Mechatronika_Pre_Synchro_Moves_Motors;
//    QAction *actionirp6_Mechatronika_Absolute_Moves_Motors;
//    QAction *actionirp6_Mechatronika_Joints;
//    QAction *actionirp6_Mechatronika_Xyz_Euler_Zyz;
//    QAction *actionirp6_Mechatronika_Xyz_Angle_Axis;
//    QAction *actionirp6_Mechatronika_Synchro_Position;
//    QAction *actionirp6_Mechatronika_Position_0;
//    QAction *actionirp6_Mechatronika_Position_1;
//    QAction *actionirp6_Mechatronika_Position_2;
//    QAction *actionirp6_Mechatronika_Tool_Xyz_Euler_Zyz;
//    QAction *actionirp6_Mechatronika_Tool_Xyz_Angle_Axis;

	QMenu *menuRobot;
	QMenu *menuFile;
	QMenu *menuSpecial;
	QMenu *menuAll_Robots;
	QMenu *menuall_Preset_Positions;
	QMenu *menuTask;
	QMenu *menuHelp;

public slots:
	// all robots menu
	void on_actionall_EDP_Load_triggered();
	void on_actionall_EDP_Unload_triggered();
	void on_actionall_Synchronisation_triggered();
	void on_actionall_Synchro_Position_triggered();
	void on_actionall_Front_Position_triggered();
	void on_actionall_Position_0_triggered();
	void on_actionall_Position_1_triggered();
	void on_actionall_Position_2_triggered();

	// task menu
	void on_actionMP_Load_triggered();
	void on_actionMP_Unload_triggered();
	void on_actionProcess_Control_triggered();
	void on_actionOpen_Configuration_triggered();
	void on_actionReload_Configuration_triggered();

	// special menu
	void on_actionClear_Console_triggered();
	void on_actionUnload_All_triggered();
	void on_actionSlay_All_triggered();

	void on_actionQuit_triggered();

private:
	void retranslateMenuBar(); //(QMainWindow *mainWindow);
	mrrocpp::ui::common::Interface *interface;

};

}

#endif
