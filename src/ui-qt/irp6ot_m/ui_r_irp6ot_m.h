// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_IRP6OT_M_H
#define __UI_R_IRP6OT_M_H

#include <QObject>
#include "../base/mainwindow.h"
#include "../base/interface.h"
#include "../base/ui.h"
#include "../irp6_m/ui_r_irp6_m.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"


namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class EcpRobot;
}

namespace irp6ot_m {

//
//
// KLASA UiRobotIrp6ot_m
//
//


class UiRobot : public QObject, public irp6_m::UiRobot
{
	Q_OBJECT

private:

public:

	UiRobot(common::Interface& _interface);

	int manage_interface();
	int	process_control_window_irp6ot_section_init(bool &wlacz_PtButton_wnd_processes_control_all_reader_start, bool &wlacz_PtButton_wnd_processes_control_all_reader_stop, bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger);

	virtual int synchronise();

	int move_to_synchro_position();
	int move_to_front_position();
	int move_to_preset_position(int variant);

	int create_ui_ecp_robot();
	int edp_create_int_extra_operations();

	int ui_get_edp_pid();
	void ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l);
	typedef int (mrrocpp::ui::irp6ot_m::UiRobot::*intUiRobotFunctionPointer)();


public slots:
	//void on_actionirp6ot_m_EDP_Load_triggered();
	void on_actionirp6ot_m_EDP_Unload_triggered();

//	void on_actionirp6ot_m_Synchronisation_triggered();
	void on_actionirp6ot_m_Pre_Synchro_Moves_Motors_triggered();

	void on_actionirp6ot_m_Absolute_Moves_Motors_triggered();
	void on_actionirp6ot_m_Joints_triggered();
	void on_actionirp6ot_m_Absolute_Moves_Xyz_Euler_Zyz_triggered();
	void on_actionirp6ot_m_Absolute_Moves_Xyz_Angle_Axis_triggered();

	void on_actionirp6ot_m_Relative_Xyz_Angle_Axis_triggered();

//	void on_actionirp6ot_m_Synchro_Position_triggered();
//	void on_actionirp6ot_m_Front_Position_triggered();
//	void on_actionirp6ot_m_Position_0_triggered();
//	void on_actionirp6ot_m_Position_1_triggered();
//	void on_actionirp6ot_m_Position_2_triggered();

	void on_actionirp6ot_m_Tool_Xyz_Euler_Zyz_triggered();
	void on_actionirp6ot_m_Tool_Xyz_Angle_Axis_triggered();

};

}
} //namespace ui
} //namespace mrrocpp

#endif

