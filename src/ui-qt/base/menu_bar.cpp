#include <QApplication>
#include "menu_bar.h"
#include "interface.h"
#include "menu_bar_action.h"
#include "mainwindow.h"
#include <QFileDialog>
#include "allrobots.h"
#include "mp.h"

namespace Ui {

MenuBar::MenuBar(mrrocpp::ui::common::Interface* iface, QWidget *parent) :
		QMenuBar(parent), interface(iface)
{

	//setupMenuBar(MainWindow);
	//signalMapper = new QSignalMapper(this);
	//connect(signalMapper, SIGNAL(mapped(QString)))
}

void MenuBar::setupMenuBar(QMainWindow *mainWindow)
{
	actionQuit = new QAction(mainWindow);
	actionQuit->setObjectName(QString::fromUtf8("actionQuit"));
	actionClear_Console = new QAction(mainWindow);
	actionClear_Console->setObjectName(QString::fromUtf8("actionClear_Console"));
	actionEDP_load_smb = new QAction(mainWindow);
	actionEDP_load_smb->setObjectName(QString::fromUtf8("actionEDP_load_smb"));
	actionUnload_All = new QAction(mainWindow);
	actionUnload_All->setObjectName(QString::fromUtf8("actionUnload_All"));
	actionSlay_All = new QAction(mainWindow);
	actionSlay_All->setObjectName(QString::fromUtf8("actionSlay_All"));
	actionAbout = new QAction(mainWindow);
	actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
	actionall_EDP_Load = new QAction(mainWindow);
	actionall_EDP_Load->setObjectName(QString::fromUtf8("actionall_EDP_Load"));
	actionall_EDP_Unload = new QAction(mainWindow);
	actionall_EDP_Unload->setObjectName(QString::fromUtf8("actionall_EDP_Unload"));
	actionall_Synchronisation = new QAction(mainWindow);
	actionall_Synchronisation->setObjectName(QString::fromUtf8("actionall_Synchronisation"));
	actionall_Synchro_Position = new QAction(mainWindow);
	actionall_Synchro_Position->setObjectName(QString::fromUtf8("actionall_Synchro_Position"));
	actionall_Front_Position = new QAction(mainWindow);
	actionall_Front_Position->setObjectName(QString::fromUtf8("actionall_Front_Position"));
	actionall_Position_0 = new QAction(mainWindow);
	actionall_Position_0->setObjectName(QString::fromUtf8("actionall_Position_0"));
	actionall_Position_1 = new QAction(mainWindow);
	actionall_Position_1->setObjectName(QString::fromUtf8("actionall_Position_1"));
	actionall_Position_2 = new QAction(mainWindow);
	actionall_Position_2->setObjectName(QString::fromUtf8("actionall_Position_2"));
	actionMP_Load = new QAction(mainWindow);
	actionMP_Load->setObjectName(QString::fromUtf8("actionMP_Load"));
	actionMP_Unload = new QAction(mainWindow);
	actionMP_Unload->setObjectName(QString::fromUtf8("actionMP_Unload"));
	actionProcess_Control = new QAction(mainWindow);
	actionProcess_Control->setObjectName(QString::fromUtf8("actionProcess_Control"));
	actionOpen_Configuration = new QAction(mainWindow);
	actionOpen_Configuration->setObjectName(QString::fromUtf8("actionOpen_Configuration"));

	actionReload_Configuration = new QAction(mainWindow);
	actionReload_Configuration->setObjectName(QString::fromUtf8("actionReload_Configuration"));

//    actionspeaker_EDP_Load = new QAction(mainWindow);
//    actionspeaker_EDP_Load->setObjectName(QString::fromUtf8("actionspeaker_EDP_Load"));
//    actionspeaker_EDP_Unload = new QAction(mainWindow);
//    actionspeaker_EDP_Unload->setObjectName(QString::fromUtf8("actionspeaker_EDP_Unload"));
//    actionspeaker_Play = new QAction(mainWindow);
//    actionspeaker_Play->setObjectName(QString::fromUtf8("actionspeaker_Play"));
//    actionspeaker_Sound_0 = new QAction(mainWindow);
//    actionspeaker_Sound_0->setObjectName(QString::fromUtf8("actionspeaker_Sound_0"));
//    actionspeaker_Sound_1 = new QAction(mainWindow);
//    actionspeaker_Sound_1->setObjectName(QString::fromUtf8("actionspeaker_Sound_1"));
//    actionspeaker_Sound_2 = new QAction(mainWindow);
//    actionspeaker_Sound_2->setObjectName(QString::fromUtf8("actionspeaker_Sound_2"));
//    actionirp6_Mechatronika_EDP_Load = new QAction(mainWindow);
//    actionirp6_Mechatronika_EDP_Load->setObjectName(QString::fromUtf8("actionirp6_Mechatronika_EDP_Load"));
//    actionirp6_Mechatronika_EDP_Unload = new QAction(mainWindow);
//    actionirp6_Mechatronika_EDP_Unload->setObjectName(QString::fromUtf8("actionirp6_Mechatronika_EDP_Unload"));
//    actionirp6_Mechatronika_Synchronization = new QAction(mainWindow);
//    actionirp6_Mechatronika_Synchronization->setObjectName(QString::fromUtf8("actionirp6_Mechatronika_Synchronization"));
//    actionirp6_Mechatronika_Pre_Synchro_Moves_Motors = new QAction(mainWindow);
//    actionirp6_Mechatronika_Pre_Synchro_Moves_Motors->setObjectName(QString::fromUtf8("actionirp6_Mechatronika_Pre_Synchro_Moves_Motors"));
//    actionirp6_Mechatronika_Absolute_Moves_Motors = new QAction(mainWindow);
//    actionirp6_Mechatronika_Absolute_Moves_Motors->setObjectName(QString::fromUtf8("actionirp6_Mechatronika_Absolute_Moves_Motors"));
//    actionirp6_Mechatronika_Joints = new QAction(mainWindow);
//    actionirp6_Mechatronika_Joints->setObjectName(QString::fromUtf8("actionirp6_Mechatronika_Joints"));
//    actionirp6_Mechatronika_Xyz_Euler_Zyz = new QAction(mainWindow);
//    actionirp6_Mechatronika_Xyz_Euler_Zyz->setObjectName(QString::fromUtf8("actionirp6_Mechatronika_Xyz_Euler_Zyz"));
//    actionirp6_Mechatronika_Xyz_Angle_Axis = new QAction(mainWindow);
//    actionirp6_Mechatronika_Xyz_Angle_Axis->setObjectName(QString::fromUtf8("actionirp6_Mechatronika_Xyz_Angle_Axis"));
//    actionirp6_Mechatronika_Synchro_Position = new QAction(mainWindow);
//    actionirp6_Mechatronika_Synchro_Position->setObjectName(QString::fromUtf8("actionirp6_Mechatronika_Synchro_Position"));
//    actionirp6_Mechatronika_Position_0 = new QAction(mainWindow);
//    actionirp6_Mechatronika_Position_0->setObjectName(QString::fromUtf8("actionirp6_Mechatronika_Position_0"));
//    actionirp6_Mechatronika_Position_1 = new QAction(mainWindow);
//    actionirp6_Mechatronika_Position_1->setObjectName(QString::fromUtf8("actionirp6_Mechatronika_Position_1"));
//    actionirp6_Mechatronika_Position_2 = new QAction(mainWindow);
//    actionirp6_Mechatronika_Position_2->setObjectName(QString::fromUtf8("actionirp6_Mechatronika_Position_2"));
//    actionirp6_Mechatronika_Tool_Xyz_Euler_Zyz = new QAction(mainWindow);
//    actionirp6_Mechatronika_Tool_Xyz_Euler_Zyz->setObjectName(QString::fromUtf8("actionirp6_Mechatronika_Tool_Xyz_Euler_Zyz"));
//    actionirp6_Mechatronika_Tool_Xyz_Angle_Axis = new QAction(mainWindow);
//    actionirp6_Mechatronika_Tool_Xyz_Angle_Axis->setObjectName(QString::fromUtf8("actionirp6_Mechatronika_Tool_Xyz_Angle_Axis"));

//    centralWidget = new QWidget(mainWindow);
//    centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
//    mainWindow->setCentralWidget(centralWidget);

	this->setObjectName(QString::fromUtf8("this"));
	this->setGeometry(QRect(0, 0, 1000, 25));
	menuRobot = new QMenu(this);
	menuRobot->setObjectName(QString::fromUtf8("menuRobot"));

	menuFile = new QMenu(this);
	menuFile->setObjectName(QString::fromUtf8("menuFile"));
	menuSpecial = new QMenu(this);
	menuSpecial->setObjectName(QString::fromUtf8("menuSpecial"));
	menuAll_Robots = new QMenu(this);
	menuAll_Robots->setObjectName(QString::fromUtf8("menuAll_Robots"));
	menuall_Preset_Positions = new QMenu(menuAll_Robots);
	menuall_Preset_Positions->setObjectName(QString::fromUtf8("menuall_Preset_Positions"));
	menuTask = new QMenu(this);
	menuTask->setObjectName(QString::fromUtf8("menuTask"));
	menuHelp = new QMenu(this);
	menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
	mainWindow->setMenuBar(this);

	this->addAction(menuFile->menuAction());
	this->addAction(menuRobot->menuAction());
	this->addAction(menuAll_Robots->menuAction());
	this->addAction(menuTask->menuAction());
	this->addAction(menuSpecial->menuAction());
	this->addAction(menuHelp->menuAction());

//	menuRobot->addSeparator();
//	menuRobot->addSeparator();
//	menuRobot->addSeparator();

	menuFile->addAction(actionQuit);
	menuSpecial->addAction(actionClear_Console);
	menuSpecial->addSeparator();
	menuSpecial->addAction(actionUnload_All);
	menuSpecial->addAction(actionSlay_All);
	menuAll_Robots->addAction(actionall_EDP_Load);
	menuAll_Robots->addAction(actionall_EDP_Unload);
	menuAll_Robots->addSeparator();
	menuAll_Robots->addAction(actionall_Synchronisation);
	menuAll_Robots->addAction(menuall_Preset_Positions->menuAction());
	menuall_Preset_Positions->addAction(actionall_Synchro_Position);
	menuall_Preset_Positions->addAction(actionall_Front_Position);
	menuall_Preset_Positions->addAction(actionall_Position_0);
	menuall_Preset_Positions->addAction(actionall_Position_1);
	menuall_Preset_Positions->addAction(actionall_Position_2);
	menuTask->addAction(actionMP_Load);
	menuTask->addAction(actionMP_Unload);
	menuTask->addSeparator();
	menuTask->addAction(actionProcess_Control);
	menuTask->addAction(actionOpen_Configuration);
	menuTask->addAction(actionReload_Configuration);
	menuHelp->addAction(actionAbout);
	retranslateMenuBar();

}

void MenuBar::retranslateMenuBar()
{
	actionQuit->setText(QApplication::translate("MainWindow", "&Quit", 0, QApplication::UnicodeUTF8));
	actionClear_Console->setText(QApplication::translate("MainWindow", "&Clear Console", 0, QApplication::UnicodeUTF8));
	actionEDP_load_smb->setText(QApplication::translate("MainWindow", "EDP load", 0, QApplication::UnicodeUTF8));
	actionUnload_All->setText(QApplication::translate("MainWindow", "&Unload All", 0, QApplication::UnicodeUTF8));
	actionSlay_All->setText(QApplication::translate("MainWindow", "&Slay All", 0, QApplication::UnicodeUTF8));
	actionAbout->setText(QApplication::translate("MainWindow", "&About", 0, QApplication::UnicodeUTF8));
	actionall_EDP_Load->setText(QApplication::translate("MainWindow", "EDP &Load", 0, QApplication::UnicodeUTF8));
	actionall_EDP_Unload->setText(QApplication::translate("MainWindow", "EDP &Unload", 0, QApplication::UnicodeUTF8));
	actionall_Synchronisation->setText(QApplication::translate("MainWindow", "&Synchronisation", 0, QApplication::UnicodeUTF8));
	actionall_Synchro_Position->setText(QApplication::translate("MainWindow", "&Synchro Position", 0, QApplication::UnicodeUTF8));
	actionall_Front_Position->setText(QApplication::translate("MainWindow", "&Front Position", 0, QApplication::UnicodeUTF8));
	actionall_Position_0->setText(QApplication::translate("MainWindow", "Position &0", 0, QApplication::UnicodeUTF8));
	actionall_Position_1->setText(QApplication::translate("MainWindow", "Position &1", 0, QApplication::UnicodeUTF8));
	actionall_Position_2->setText(QApplication::translate("MainWindow", "Position &2", 0, QApplication::UnicodeUTF8));
	actionMP_Load->setText(QApplication::translate("MainWindow", "MP &Load", 0, QApplication::UnicodeUTF8));
	actionMP_Unload->setText(QApplication::translate("MainWindow", "MP &Unload", 0, QApplication::UnicodeUTF8));
	actionProcess_Control->setText(QApplication::translate("MainWindow", "&Process Control", 0, QApplication::UnicodeUTF8));
	actionOpen_Configuration->setText(QApplication::translate("MainWindow", "&Open Configuration", 0, QApplication::UnicodeUTF8));
	actionReload_Configuration->setText(QApplication::translate("MainWindow", "&Reload &Configuration", 0, QApplication::UnicodeUTF8));

	menuRobot->setTitle(QApplication::translate("MainWindow", "&Robot", 0, QApplication::UnicodeUTF8));
	menuFile->setTitle(QApplication::translate("MainWindow", "&File", 0, QApplication::UnicodeUTF8));
	menuSpecial->setTitle(QApplication::translate("MainWindow", "&Special", 0, QApplication::UnicodeUTF8));
	menuAll_Robots->setTitle(QApplication::translate("MainWindow", "&All robots", 0, QApplication::UnicodeUTF8));
	menuall_Preset_Positions->setTitle(QApplication::translate("MainWindow", "Pr&eset Positions", 0, QApplication::UnicodeUTF8));
	menuTask->setTitle(QApplication::translate("MainWindow", "&Task", 0, QApplication::UnicodeUTF8));
	menuHelp->setTitle(QApplication::translate("MainWindow", "&Help", 0, QApplication::UnicodeUTF8));

	makeConnections();
}

void MenuBar::makeConnections()
{
	printf("connected");
	connect(actionall_EDP_Load, SIGNAL(triggered()), this, SLOT(on_actionall_EDP_Load_triggered()), Qt::QueuedConnection);
	connect(actionall_EDP_Unload, SIGNAL(triggered()), this, SLOT(on_actionall_EDP_Unload_triggered()), Qt::QueuedConnection);
	connect(actionall_Synchronisation, SIGNAL(triggered()), this, SLOT(on_actionall_Synchronisation_triggered()), Qt::QueuedConnection);
	connect(actionall_Synchro_Position, SIGNAL(triggered()), this, SLOT(on_actionall_Synchro_Position_triggered()), Qt::QueuedConnection);
	connect(actionall_Front_Position, SIGNAL(triggered()), this, SLOT(on_actionall_Front_Position_triggered()), Qt::QueuedConnection);
	connect(actionall_Position_0, SIGNAL(triggered()), this, SLOT(on_actionall_Position_0_triggered()), Qt::QueuedConnection);
	connect(actionall_Position_1, SIGNAL(triggered()), this, SLOT(on_actionall_Position_1_triggered()), Qt::QueuedConnection);
	connect(actionall_Position_2, SIGNAL(triggered()), this, SLOT(on_actionall_Position_2_triggered()), Qt::QueuedConnection);
	connect(actionMP_Load, SIGNAL(triggered()), this, SLOT(on_actionMP_Load_triggered()), Qt::QueuedConnection);
	connect(actionMP_Unload, SIGNAL(triggered()), this, SLOT(on_actionMP_Unload_triggered()), Qt::QueuedConnection);
	connect(actionProcess_Control, SIGNAL(triggered()), this, SLOT(on_actionProcess_Control_triggered()), Qt::QueuedConnection);
	connect(actionOpen_Configuration, SIGNAL(triggered()), this, SLOT(on_actionOpen_Configuration_triggered()), Qt::QueuedConnection);
	connect(actionReload_Configuration, SIGNAL(triggered()), this, SLOT(on_actionReload_Configuration_triggered()), Qt::QueuedConnection);
	connect(actionClear_Console, SIGNAL(triggered()), this, SLOT(on_actionClear_Console_triggered()), Qt::QueuedConnection);
	connect(actionUnload_All, SIGNAL(triggered()), this, SLOT(on_actionUnload_All_triggered()), Qt::QueuedConnection);
	connect(actionSlay_All, SIGNAL(triggered()), this, SLOT(on_actionSlay_All_triggered()), Qt::QueuedConnection);
	connect(actionQuit, SIGNAL(triggered()), this, SLOT(on_actionQuit_triggered()), Qt::QueuedConnection);
}

void MenuBar::on_actionall_EDP_Load_triggered()
{
	printf("action all edp triggered");
	interface->all_robots->EDP_all_robots_create();
}

void MenuBar::on_actionall_EDP_Unload_triggered()
{
	interface->all_robots->EDP_all_robots_slay();
}

void MenuBar::on_actionall_Synchronisation_triggered()
{
	interface->all_robots->EDP_all_robots_synchronise();
}

void MenuBar::on_actionall_Synchro_Position_triggered()
{
	interface->all_robots->move_to_synchro_position();
}

void MenuBar::on_actionall_Front_Position_triggered()
{
	interface->all_robots->move_to_front_position();
}

void MenuBar::on_actionall_Position_0_triggered()
{
	interface->all_robots->move_to_preset_position(0);
}

void MenuBar::on_actionall_Position_1_triggered()
{
	interface->all_robots->move_to_preset_position(1);
}

void MenuBar::on_actionall_Position_2_triggered()
{
	interface->all_robots->move_to_preset_position(2);
}

// task menu

void MenuBar::on_actionMP_Load_triggered()
{
	interface->mp->MPup();
}

void MenuBar::on_actionMP_Unload_triggered()
{
	interface->mp->MPslay();
}

void MenuBar::on_actionProcess_Control_triggered()
{
	interface->raise_process_control_window();
}

void MenuBar::on_actionOpen_Configuration_triggered()
{
	/*
	 QFileDialog dialog;
	 if (dialog.exec()) {
	 // ...
	 }
	 */
	try {
		QString fileName;

		std::string mrrocpp_current_config_full_path = interface->mrrocpp_root_local_path + interface->config_file;
		interface->ui_msg->message(mrrocpp_current_config_full_path);

		fileName =
				QFileDialog::getOpenFileName(this, tr("Choose configuration file or die"), mrrocpp_current_config_full_path.c_str(), tr("Image Files (*.ini)"));
		if (fileName.length() > 0) {
			std::string str_fullpath = fileName.toStdString();

			interface->config_file = str_fullpath.substr(str_fullpath.rfind(interface->mrrocpp_root_local_path)
					+ interface->mrrocpp_root_local_path.length());
			interface->reload_whole_configuration();
			interface->set_default_configuration_file_name();
		}
	}

	catch (...) {

	}

}

void MenuBar::on_actionReload_Configuration_triggered()
{

	try {

		interface->reload_whole_configuration();

	}

	catch (...) {

	}

}

// special menu

void MenuBar::on_actionClear_Console_triggered()
{
	interface->get_main_window()->clear_console();
}

void MenuBar::on_actionUnload_All_triggered()
{
	interface->unload_all();
}

void MenuBar::on_actionSlay_All_triggered()
{
	interface->slay_all();
}

void MenuBar::on_actionQuit_triggered()
{
	interface->UI_close();
}

}
