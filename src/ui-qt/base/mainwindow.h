#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QCloseEvent>
#include <pthread.h>

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

	void ui_notification();
	//void enable_menu_item(bool _active, QWidget *_menu_item);
	void enable_menu_item(bool _enable, int _num_of_menus, QWidget *_menu_item, ...);
	void enable_menu_item(bool _enable, int _num_of_menus, QAction *_menu_item, ...);
	//void disable_menu_item(int _num_of_menus, ...);
	void raise_process_control_window();
	void raise_ui_ecp_window();
	void get_lineEdit_position(double* val, int number_of_servos);

	Ui::MainWindow * get_ui();

	void closeEvent(QCloseEvent * event);

private:
	Ui::MainWindow *ui;
	mrrocpp::ui::common::Interface& interface;
	QTimer *timer;

	pthread_t main_thread_id;

signals:
	void ui_notification_signal();
	void enable_menu_item_signal(QWidget *_menu_item, bool _active);
	void enable_menu_item_signal(QAction *_menu_item, bool _active);
	void raise_process_control_window_signal();
	void raise_ui_ecp_window_signal();

private slots:

	void on_timer_slot();

	void ui_notification_slot();
	void raise_process_control_window_slot();
	void raise_ui_ecp_window_slot();
	void enable_menu_item_slot(QWidget *_menu_item, bool _active);
	void enable_menu_item_slot(QAction *_menu_item, bool _active);
	// menus

	// file menu
	void on_actionQuit_triggered();

	// robot menu

	//irp6ot_m menu
	void on_actionirp6ot_m_EDP_Load_triggered();
	void on_actionirp6ot_m_EDP_Unload_triggered();

	void on_actionirp6ot_m_Synchronisation_triggered();
	void on_actionirp6ot_m_Pre_Synchro_Moves_Motors_triggered();

	void on_actionirp6ot_m_Absolute_Moves_Motors_triggered();
	void on_actionirp6ot_m_Joints_triggered();
	void on_actionirp6ot_m_Absolute_Moves_Xyz_Euler_Zyz_triggered();
	void on_actionirp6ot_m_Absolute_Moves_Xyz_Angle_Axis_triggered();

	void on_actionirp6ot_m_Relative_Xyz_Angle_Axis_triggered();

	void on_actionirp6ot_m_Synchro_Position_triggered();
	void on_actionirp6ot_m_Front_Position_triggered();
	void on_actionirp6ot_m_Position_0_triggered();
	void on_actionirp6ot_m_Position_1_triggered();
	void on_actionirp6ot_m_Position_2_triggered();

	void on_actionirp6ot_m_Tool_Xyz_Euler_Zyz_triggered();
	void on_actionirp6ot_m_Tool_Xyz_Angle_Axis_triggered();

	//irp6ot_tfg
	void on_actionirp6ot_tfg_EDP_Load_triggered();
	void on_actionirp6ot_tfg_EDP_Unload_triggered();

	void on_actionirp6ot_tfg_Synchronization_triggered();
	void on_actionirp6ot_tfg_Move_triggered();

	void on_actionirp6ot_tfg_Synchro_Position_triggered();
	void on_actionirp6ot_tfg_Position_0_triggered();
	void on_actionirp6ot_tfg_Position_1_triggered();
	void on_actionirp6ot_tfg_Position_2_triggered();

	//irp6p_m menu
	void on_actionirp6p_m_EDP_Load_triggered();
	void on_actionirp6p_m_EDP_Unload_triggered();

	void on_actionirp6p_m_Synchronisation_triggered();
	void on_actionirp6p_m_Pre_Synchro_Moves_Motors_triggered();

	void on_actionirp6p_m_Absolute_Moves_Motors_triggered();
	void on_actionirp6p_m_Joints_triggered();
	void on_actionirp6p_m_Absolute_Moves_Xyz_Euler_Zyz_triggered();
	void on_actionirp6p_m_Absolute_Moves_Xyz_Angle_Axis_triggered();

	void on_actionirp6p_m_Xyz_Relative_Moves_Angle_Axis_triggered();

	void on_actionirp6p_m_Synchro_Position_triggered();
	void on_actionirp6p_m_Front_Position_triggered();
	void on_actionirp6p_m_Position_0_triggered();
	void on_actionirp6p_m_Position_1_triggered();
	void on_actionirp6p_m_Position_2_triggered();

	void on_actionirp6p_m_Tool_Xyz_Euler_Zyz_triggered();
	void on_actionirp6p_m_Tool_Xyz_Angle_Axis_triggered();

	//irp6p_tfg
	void on_actionirp6p_tfg_EDP_Load_triggered();
	void on_actionirp6p_tfg_EDP_Unload_triggered();

	void on_actionirp6p_tfg_Synchronization_triggered();
	void on_actionirp6p_tfg_Move_triggered();

	void on_actionirp6p_tfg_Synchro_Position_triggered();
	void on_actionirp6p_tfg_Position_0_triggered();
	void on_actionirp6p_tfg_Position_1_triggered();
	void on_actionirp6p_tfg_Position_2_triggered();

	//conveyor
	void on_actionconveyor_EDP_Load_triggered();
	void on_actionconveyor_EDP_Unload_triggered();

	void on_actionconveyor_Synchronization_triggered();
	void on_actionconveyor_Move_triggered();

	void on_actionconveyor_Synchro_Position_triggered();
	void on_actionconveyor_Position_0_triggered();
	void on_actionconveyor_Position_1_triggered();
	void on_actionconveyor_Position_2_triggered();

	// birdhand menu
	void on_actionbirdhand_EDP_Load_triggered();
	void on_actionbirdhand_EDP_Unload_triggered();

	void on_actionbirdhand_Command_triggered();
	void on_actionbirdhand_Configuration_triggered();

	// sarkofag menu
	void on_actionsarkofag_EDP_Load_triggered();
	void on_actionsarkofag_EDP_Unload_triggered();

	void on_actionsarkofag_Synchronisation_triggered();
	void on_actionsarkofag_Move_triggered();

	void on_actionsarkofag_Synchro_Position_triggered();
	void on_actionsarkofag_Front_Position_triggered();
	void on_actionsarkofag_Position_0_triggered();
	void on_actionsarkofag_Position_1_triggered();
	void on_actionsarkofag_Position_2_triggered();

	void on_actionsarkofag_Servo_Algorithm_triggered();

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

	void on_actionspkm_Clear_Fault_triggered();

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
	void on_actionConfiguration_triggered();

	// special menu
	void on_actionClear_Console_triggered();
	void on_actionUnload_All_triggered();
	void on_actionSlay_All_triggered();

};

#endif // MAINWINDOW_H
