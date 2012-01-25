//#include "ui_ecp_r_irp6_m.h"
#include "ui_r_irp6_m.h"

//#include "ui/src/ui_ecp_r_single_motor.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common012.h"
#include "wgt_irp6_m_tool_euler.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"

//const int wgt_irp6_m_tool_euler::angle_axis_number = 6;

wgt_irp6_m_tool_euler::wgt_irp6_m_tool_euler(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent) :
		WgtToolBase(_widget_label, _interface, _robot, parent)
{
	//ui.setupUi(this);
	robot = dynamic_cast <mrrocpp::ui::irp6_m::UiRobot *>(_robot);

}

wgt_irp6_m_tool_euler::~wgt_irp6_m_tool_euler()
{

}

void wgt_irp6_m_tool_euler::init()
{

	try {

		if (robot->state.edp.pid != -1) {
			if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				synchro_depended_widgets_disable(false);

				robot->ui_ecp_robot->read_tool_xyz_euler_zyz(tool_vector); // co tutaj ma być?

				for (int i = 0; i < angle_axis_number; i++) {
					current_pos_spin_boxes[i]->setValue(tool_vector[i]);

				}

			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				synchro_depended_widgets_disable(true);

			}
		}

	} // end try
	CATCH_SECTION_UI_PTR
}

void wgt_irp6_m_tool_euler::get_desired_position()
{

	if (robot->state.edp.pid != -1) {

		if (robot->state.edp.is_synchronised) {

			for (int i = 0; i < angle_axis_number; i++) {
				tool_vector[i] = desired_pos_spin_boxes[i]->value();
			}
		} else {

			for (int i = 0; i < angle_axis_number; i++) {
				tool_vector[i] = 0.0;
			}
		}
	}
}

void wgt_irp6_m_tool_euler::move_it()
{

	// wychwytania ew. bledow ECP::robot
	try {

		if (robot->state.edp.pid != -1) {

			//robot->ui_ecp_robot->move_tool_euler(robot->desired_pos);

			//robot->ui_ecp_robot->move_tool_euler(robot->desired_pos);
			robot->ui_ecp_robot->set_tool_xyz_euler_zyz(tool_vector);

			//robot->ui_ecp_robot->interface.irp6_m->ui_ecp_robot->move_tool_euler(robot->desired_pos);
			//interface.irp6_m->ui_ecp_robot->move_tool_euler(interface.irp6_m->desired_pos);
			//interface.irp6_m->ui_ecp_robot->move_tool_euler(interface.irp6_m->desired_pos);
			//robot->ui_ecp_robot->move_tool_euler(robot->desired_pos);

			if ((robot->state.edp.is_synchronised) /* TR && (is_open)*/) { // by Y o dziwo nie dziala poprawnie 	 if (robot->state.edp.is_synchronised)
				for (int i = 0; i < angle_axis_number; i++) {
					desired_pos_spin_boxes[i]->setValue(tool_vector[i]);
				}
				init();
			}
		} // end if (robot->state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI_PTR
}

