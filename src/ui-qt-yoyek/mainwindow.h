#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

namespace Ui {
class MainWindow;
}

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
}
}

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
	explicit MainWindow(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~MainWindow();

	void ui_notification(QString _string, QColor _color);
	void raise_process_control_window();

private:
	Ui::MainWindow *ui;
	mrrocpp::ui::common::Interface& interface;
	QTimer *timer;

	signals:
	void ui_notification_signal(QString _string, QColor _color);
	void raise_process_control_window_signal();

private slots:

	void on_timer_slot();

	void ui_notification_slot(QString _string, QColor _color);
	void raise_process_control_window_slot();

	// menus

	// file menu
	void on_actionQuit_triggered();

	// robot menu

	//irp6ot_m menu
	void on_actionirp6ot_m_EDP_Load_triggered();
	void on_actionirp6ot_m_EDP_Unload_triggered();

	void on_actionirp6ot_m_Synchronisation_triggered();

	void on_actionirp6ot_m_Synchro_Position_triggered();
	void on_actionirp6ot_m_Front_Position_triggered();
	void on_actionirp6ot_m_Position_0_triggered();
	void on_actionirp6ot_m_Position_1_triggered();
	void on_actionirp6ot_m_Position_2_triggered();

	//irp6p_m menu
	void on_actionirp6p_m_EDP_Load_triggered();
	void on_actionirp6p_m_EDP_Unload_triggered();

	void on_actionirp6p_m_Synchronisation_triggered();

	void on_actionirp6p_m_Synchro_Position_triggered();
	void on_actionirp6p_m_Front_Position_triggered();
	void on_actionirp6p_m_Position_0_triggered();
	void on_actionirp6p_m_Position_1_triggered();
	void on_actionirp6p_m_Position_2_triggered();

	// spkm menu
	void on_actionspkm_EDP_Load_triggered();
	void on_actionspkm_EDP_Unload_triggered();

	void on_actionspkm_Synchronisation_triggered();
	void on_actionspkm_Motors_triggered();

	void on_actionspkm_Motors_post_triggered();
	void on_actionspkm_Joints_triggered();
	void on_actionspkm_External_triggered();

	void on_actionspkm_Synchro_Position_triggered();
	void on_actionspkm_Front_Position_triggered();
	void on_actionspkm_Position_0_triggered();
	void on_actionspkm_Position_1_triggered();
	void on_actionspkm_Position_2_triggered();

	// smb menu
	void on_actionsmb_EDP_Load_triggered();
	void on_actionsmb_EDP_Unload_triggered();

	// shead menu
	void on_actionshead_EDP_Load_triggered();
	void on_actionshead_EDP_Unload_triggered();

	// polycrank menu
	void on_actionpolycrank_EDP_Load_triggered();
	void on_actionpolycrank_EDP_Unload_triggered();
	void on_actionpolycrank_Move_Joints_triggered();

	// all robots menu
	void on_actionall_EDP_Load_triggered();
	void on_actionall_EDP_Uload_triggered();
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
	void on_actionConfiguration_triggered();

	// special menu
	void on_actionClear_Console_triggered();
	void on_actionUnload_All_triggered();
	void on_actionSlay_All_triggered();

};

#endif // MAINWINDOW_H
