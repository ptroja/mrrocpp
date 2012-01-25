//#include "ui_ecp_r_irp6_m.h"
#include <QSignalMapper>
#include "ui_r_irp6_m.h"

//#include "ui/src/ui_ecp_r_single_motor.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common012.h"
#include "wgt_irp6_m_relative_angle_axis.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"

wgt_irp6_m_relative_angle_axis::wgt_irp6_m_relative_angle_axis(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent) :
		WgtRelativeBase(_widget_label, _interface, _robot, parent)
{
	ui.setupUi(this);
	robot = dynamic_cast <mrrocpp::ui::irp6_m::UiRobot *>(_robot);

	setup_ui(ui.gridLayout);
}

wgt_irp6_m_relative_angle_axis::~wgt_irp6_m_relative_angle_axis()
{

}

void wgt_irp6_m_relative_angle_axis::init()
{
	try {

		if (robot->state.edp.pid != -1) {
			if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				synchro_depended_widgets_disable(false);

				//robot->ui_ecp_robot->read_xyz_angle_axis(robot->current_pos);

				for (int i = 0; i < angle_axis_number; i++) {
					desired_pos_spin_boxes[i]->setValue(0.0);
					robot->desired_pos[i] = robot->current_pos[i];
				}

			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				synchro_depended_widgets_disable(true);
			}
		}
	} // end try
	CATCH_SECTION_UI_PTR
}

void wgt_irp6_m_relative_angle_axis::move_it()
{

	// wychwytania ew. bledow ECP::robot
	try {

		if (robot->state.edp.pid != -1) {

			robot->ui_ecp_robot->move_xyz_angle_axis_relative(robot->desired_pos);

			//robot->ui_ecp_robot->interface.irp6_m->ui_ecp_robot->move_relative_angle_axis(robot->desired_pos);
			//interface.irp6_m->ui_ecp_robot->move_relative_angle_axis(interface.irp6_m->desired_pos);
			//interface.irp6_m->ui_ecp_robot->move_relative_angle_axis(interface.irp6_m->desired_pos);
			//robot->ui_ecp_robot->move_relative_angle_axis(robot->desired_pos);
			/*
			 if ((robot->state.edp.is_synchronised)
			 for (int i = 0; i < aa_number; i++) {
			 doubleSpinBox_des_Vector[i]->setValue(robot->desired_pos[i]);
			 }
			 //	init();
			 }
			 */
		} // end if (robot->state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI_PTR
}

