/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "../irp6_m/wgt_irp6_m_joints.h"
#include "../irp6_m/wgt_irp6_m_motors.h"
#include "../irp6_m/wgt_irp6_m_euler.h"
#include "../irp6_m/wgt_irp6_m_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_relative_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_tool_angle_axis.h"
#include "../irp6_m/wgt_irp6_m_tool_euler.h"

#include "ui_r_irp6ot_m.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"
#include "../base/signal_dispatcher.h"


namespace mrrocpp {
namespace ui {
namespace irp6ot_m {
const std::string WGT_IRP6OT_M_JOINTS = "WGT_IRP6OT_M_JOINTS";
const std::string WGT_IRP6OT_M_MOTORS = "WGT_IRP6OT_M_MOTORS";
const std::string WGT_IRP6OT_M_ANGLE_AXIS = "WGT_IRP6OT_M_ANGLE_AXIS";
const std::string WGT_IRP6OT_M_EULER = "WGT_IRP6OT_M_EULER";
const std::string WGT_IRP6OT_M_RELATIVE_ANGLE_AXIS = "WGT_IRP6OT_M_RELATIVE_ANGLE_AXIS";
const std::string WGT_IRP6OT_M_TOOL_ANGLE_AXIS = "WGT_IRP6OT_M_TOOL_ANGLE_AXIS";
const std::string WGT_IRP6OT_M_TOOL_EULER = "WGT_IRP6OT_M_TOOL_EULER";
//
//
// KLASA UiRobot
//
//


//void UiRobot::on_actionirp6ot_m_EDP_Load_triggered()
//{
//	interface.get_main_window()->ui_robot_action(this, &UiRobot::edp_create);
//}

void UiRobot::on_actionirp6ot_m_EDP_Unload_triggered()
{
	printf("edp slay\n");
	interface.get_main_window()->ui_robot_action(this, &UiRobot::EDP_slay_int);
}

//void UiRobot::on_actionirp6ot_m_Synchronisation_triggered()
//{
////	interface.get_main_window()->ui_robot_action(interface.irp6ot_m, &UiRobot::synchronise);
//	interface.get_main_window()->ui_robot_action(this, &UiRobot::synchronise);
//}

void UiRobot::on_actionirp6ot_m_Pre_Synchro_Moves_Motors_triggered()
{
	interface.get_main_window()->open_new_window(this->wgt_motors, &wgt_base::my_open); //!!!!!!!!!!!!!!!!!!!!!!
}

void UiRobot::on_actionirp6ot_m_Absolute_Moves_Motors_triggered()
{
	interface.get_main_window()->open_new_window(this->wgt_motors, &wgt_base::my_open);
	//interface.print_on_sr("on action");
	printf("akcja wewnatrz ui_r_irp6ot_m\n");
}

void UiRobot::on_actionirp6ot_m_Joints_triggered()
{
	interface.get_main_window()->open_new_window(this->wgt_joints, &wgt_base::my_open);
}

void UiRobot::on_actionirp6ot_m_Absolute_Moves_Xyz_Euler_Zyz_triggered()
{
	interface.get_main_window()->open_new_window(this->wgt_euler, &wgt_base::my_open);
}

void UiRobot::on_actionirp6ot_m_Absolute_Moves_Xyz_Angle_Axis_triggered()
{
	interface.get_main_window()->open_new_window(this->wgt_angle_axis, &wgt_base::my_open);
}

void UiRobot::on_actionirp6ot_m_Relative_Xyz_Angle_Axis_triggered()
{
	interface.get_main_window()->open_new_window(this->wgt_relative_angle_axis, &wgt_base::my_open);
}

//void UiRobot::on_actionirp6ot_m_Synchro_Position_triggered()
//{
//	interface.get_main_window()->ui_robot_action(this, &UiRobot::move_to_synchro_position);
//}
//
//void UiRobot::on_actionirp6ot_m_Front_Position_triggered()
//{
//	interface.get_main_window()->ui_robot_action(this, &UiRobot::move_to_front_position);
//}
//
//void UiRobot::on_actionirp6ot_m_Position_0_triggered()
//{
//	interface.get_main_window()->ui_robot_action(this, &UiRobot::move_to_preset_position(0));
//}
//
//void UiRobot::on_actionirp6ot_m_Position_1_triggered()
//{
//	interface.get_main_window()->ui_robot_action(this, &UiRobot::move_to_preset_position(1));
//}
//
//void UiRobot::on_actionirp6ot_m_Position_2_triggered()
//{
//	interface.get_main_window()->ui_robot_action(this, &UiRobot::move_to_preset_position(2));
//}

void UiRobot::on_actionirp6ot_m_Tool_Xyz_Euler_Zyz_triggered()
{
	interface.get_main_window()->open_new_window(this->wgt_tool_euler, &wgt_base::my_open);
}

void UiRobot::on_actionirp6ot_m_Tool_Xyz_Angle_Axis_triggered()
{
	interface.get_main_window()->open_new_window(this->wgt_tool_angle_axis, &wgt_base::my_open);
}


int UiRobot::ui_get_edp_pid()
{
	return ui_ecp_robot->ecp->get_EDP_pid();
}

void UiRobot::ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	ui_ecp_robot->get_controller_state(robot_controller_initial_state_l);

}

int UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new ui::common::EcpRobot(*this);
	return 1;
}

int UiRobot::edp_create_int_extra_operations()
{
	wgt_motors->synchro_depended_init();
	return 1;
}

int UiRobot::move_to_synchro_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = 0.0;
	}
	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::execute_motor_motion, &(*this)));

	return 1;
}

int UiRobot::move_to_front_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.front_position[i];
	}

	//	printf(" move_to_front_position state.edp.front_position: %lf, %lf, %lf, %lf, %lf, %lf\n ", state.edp.front_position[0], state.edp.front_position[1], state.edp.front_position[2], state.edp.front_position[3], state.edp.front_position[4], state.edp.front_position[5]);

	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

int UiRobot::move_to_preset_position(int variant)
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.preset_position[variant][i];
	}
	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

UiRobot::UiRobot(common::Interface& _interface) :
	irp6_m::UiRobot(_interface, lib::irp6ot_m::ROBOT_NAME, lib::irp6ot_m::NUM_OF_SERVOS)
{
	wgt_joints = new wgt_irp6_m_joints("Irp6ot_m joints", interface, *this, interface.get_main_window());
	wgt_motors = new wgt_irp6_m_motors("Irp6ot_m motors", interface, *this, interface.get_main_window());
	wgt_angle_axis = new wgt_irp6_m_angle_axis("Irp6ot_m angle axis", interface, *this, interface.get_main_window());
	wgt_euler = new wgt_irp6_m_euler("Irp6ot_m euler", interface, *this, interface.get_main_window());
	wgt_relative_angle_axis
			= new wgt_irp6_m_relative_angle_axis("Irp6ot_m relative angle axis", interface, *this, interface.get_main_window());
	wgt_tool_angle_axis
			= new wgt_irp6_m_tool_angle_axis("Irp6ot_m tool angle axis", interface, *this, interface.get_main_window());
	wgt_tool_euler = new wgt_irp6_m_tool_euler("Irp6ot_m tool euler", interface, *this, interface.get_main_window());

	wndbase_m[WGT_IRP6OT_M_JOINTS] = wgt_joints->dwgt;
	wndbase_m[WGT_IRP6OT_M_MOTORS] = wgt_motors->dwgt;
	wndbase_m[WGT_IRP6OT_M_ANGLE_AXIS] = wgt_angle_axis->dwgt;
	wndbase_m[WGT_IRP6OT_M_EULER] = wgt_euler->dwgt;
	wndbase_m[WGT_IRP6OT_M_RELATIVE_ANGLE_AXIS] = wgt_relative_angle_axis->dwgt;
	wndbase_m[WGT_IRP6OT_M_TOOL_ANGLE_AXIS] = wgt_tool_angle_axis->dwgt;
	wndbase_m[WGT_IRP6OT_M_TOOL_EULER] = wgt_tool_euler->dwgt;


	//connect(interface.get_main_window()->get_ui()->actionirp6ot_m_EDP_Load, 					SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_EDP_Load_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_EDP_Load, 					SIGNAL(triggered()), interface.get_main_window()->getSignalDispatcher(), SLOT(on_EDP_Load_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_EDP_Unload, 					SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_EDP_Unload_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_Synchronisation, 				SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_Synchronisation_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_Pre_Synchro_Moves_Motors, 	SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_Pre_Synchro_Moves_Motors_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_Absolute_Moves_Motors, 		SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_Absolute_Moves_Motors_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_Joints, 						SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_Joints_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_Absolute_Moves_Xyz_Euler_Zyz, SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_Absolute_Moves_Xyz_Euler_Zyz_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_Absolute_Moves_Xyz_Angle_Axis,SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_Absolute_Moves_Xyz_Angle_Axis_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_Relative_Xyz_Angle_Axis, 		SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_Relative_Xyz_Angle_Axis_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_Synchro_Position, 			SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_Synchro_Position_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_Front_Position, 				SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_Front_Position_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_Position_0, 					SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_Position_0_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_Position_1, 					SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_Position_1_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_Position_2, 					SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_Position_2_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_Tool_Xyz_Euler_Zyz, 			SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_Tool_Xyz_Euler_Zyz_triggered()), Qt::AutoCompatConnection);
	connect(interface.get_main_window()->get_ui()->actionirp6ot_m_Tool_Xyz_Angle_Axis, 			SIGNAL(triggered()), this, SLOT(on_actionirp6ot_m_Tool_Xyz_Angle_Axis_triggered()), Qt::AutoCompatConnection);


}

int UiRobot::synchronise()

{

	eb.command(boost::bind(&ui::irp6ot_m::UiRobot::synchronise_int, &(*this)));

	return 1;

}

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();
	Ui::MainWindow *ui = mw->get_ui();

	switch (state.edp.state)
	{

		case -1:
			mw->enable_menu_item(false, 1, ui->menuIrp6ot_m);

			break;
		case 0:
			mw->enable_menu_item(false, 5, ui->menuirp6ot_m_Pre_Synchro_Moves, ui->menuirp6ot_m_Absolute_moves, ui->menuIrp6ot_m_Relative_Moves, ui->menuirp6ot_m_Tool, ui->menuirp6ot_m_Preset_Positions);
			mw->enable_menu_item(false, 1, ui->actionirp6ot_m_EDP_Unload);
			mw->enable_menu_item(true, 1, ui->menuIrp6ot_m);
			mw->enable_menu_item(true, 1, ui->actionirp6ot_m_EDP_Load);

			break;
		case 1:
		case 2:
			mw->enable_menu_item(true, 1, ui->menuIrp6ot_m);
			mw->enable_menu_item(true, 1, ui->actionall_EDP_Unload);

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				mw->enable_menu_item(false, 1, ui->menuirp6ot_m_Pre_Synchro_Moves);
				mw->enable_menu_item(true, 1, ui->menuall_Preset_Positions);

				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 4, ui->menuirp6ot_m_Absolute_moves, ui->menuIrp6ot_m_Relative_Moves, ui->menuirp6ot_m_Tool, ui->menuirp6ot_m_Preset_Positions);
						mw->enable_menu_item(true, 1, ui->actionirp6ot_m_EDP_Unload);
						mw->enable_menu_item(false, 1, ui->actionirp6ot_m_EDP_Load);

						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(true, 4, ui->menuirp6ot_m_Absolute_moves, ui->menuIrp6ot_m_Relative_Moves, ui->menuirp6ot_m_Tool, ui->menuirp6ot_m_Preset_Positions);
						mw->enable_menu_item(false, 2, ui->actionirp6ot_m_EDP_Load, ui->actionirp6ot_m_EDP_Unload);

						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						mw->enable_menu_item(false, 4, ui->menuirp6ot_m_Absolute_moves, ui->menuIrp6ot_m_Relative_Moves, ui->menuirp6ot_m_Tool, ui->menuirp6ot_m_Preset_Positions);

						break;
					default:
						break;
				}

			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 1, ui->menuirp6ot_m_Pre_Synchro_Moves);
				mw->enable_menu_item(true, 1, ui->actionirp6ot_m_EDP_Unload);
				mw->enable_menu_item(false, 1, ui->actionirp6ot_m_EDP_Load);

			}
			break;
		default:
			break;

	}

	return 1;
}

// aktualizacja ustawien przyciskow
int UiRobot::process_control_window_irp6ot_section_init(bool &wlacz_PtButton_wnd_processes_control_all_reader_start, bool &wlacz_PtButton_wnd_processes_control_all_reader_stop, bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger)
{

	if (state.edp.state <= 0) {// edp wylaczone

		/* TR
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_start);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_stop);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_trigger);
		 */
	} else if (state.edp.state == 1) {// edp wlaczone reader czeka na start
		wlacz_PtButton_wnd_processes_control_all_reader_start = true;
		/* TR
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_start);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_stop);
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_trigger);
		 */
	} else if (state.edp.state == 2) {// edp wlaczone reader czeka na stop
		wlacz_PtButton_wnd_processes_control_all_reader_stop = true;
		wlacz_PtButton_wnd_processes_control_all_reader_trigger = true;
		/* TR
		 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_start);
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_stop);
		 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_reader_trigger);
		 */
	}

	state.edp.last_state = state.edp.state;

	return 1;

}

}
} //namespace ui
} //namespace mrrocpp
