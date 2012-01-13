//#include "ui_ecp_r_irp6_m.h"
#include "ui_r_irp6_m.h"

//#include "ui/src/ui_ecp_r_single_motor.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common012.h"
#include "wgt_irp6_m_angle_axis.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"

wgt_irp6_m_angle_axis::wgt_irp6_m_angle_axis(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent) :
		WgtAbsoluteBase(_widget_label, _interface, _robot, parent)
{
	ui.setupUi(this);
	robot = dynamic_cast <mrrocpp::ui::irp6_m::UiRobot *>(_robot);

	ui.label_axis_1->setText("x");
	ui.label_axis_2->setText("y");
	ui.label_axis_3->setText("z");
	ui.label_axis_4->setText("ax");
	ui.label_axis_5->setText("ay");
	ui.label_axis_6->setText("az");
	ui.label_axis_7->hide();

	setup_ui(ui.gridLayout, 6);
}

void wgt_irp6_m_angle_axis::setup_ui(QGridLayout *layout, int _rows_number)
{
	WgtAbsoluteBase::setup_ui(layout, _rows_number);

}

wgt_irp6_m_angle_axis::~wgt_irp6_m_angle_axis()
{

}

void wgt_irp6_m_angle_axis::init()
{

	try {

		if (robot->state.edp.pid != -1) {
			if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				synchro_depended_widgets_disable(false);

				robot->ui_ecp_robot->read_xyz_angle_axis(robot->current_pos);

				for (int i = 0; i < 6; i++) {
					current_pos_spin_boxes[i]->setValue(robot->current_pos[i]);
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

void wgt_irp6_m_angle_axis::move_it()
{

	// wychwytania ew. bledow ECP::robot
	try {

		if (robot->state.edp.pid != -1) {

			//robot->ui_ecp_robot->move_angle_axis(robot->desired_pos);

			//robot->ui_ecp_robot->move_angle_axis(robot->desired_pos);
			robot->ui_ecp_robot->move_xyz_angle_axis(robot->desired_pos);

			//robot->ui_ecp_robot->interface.irp6_m->ui_ecp_robot->move_angle_axis(robot->desired_pos);
			//interface.irp6_m->ui_ecp_robot->move_angle_axis(interface.irp6_m->desired_pos);
			//interface.irp6_m->ui_ecp_robot->move_angle_axis(interface.irp6_m->desired_pos);
			//robot->ui_ecp_robot->move_angle_axis(robot->desired_pos);

			if ((robot->state.edp.is_synchronised) /* TR && (is_open)*/) { // by Y o dziwo nie dziala poprawnie 	 if (robot->state.edp.is_synchronised)
				for (int i = 0; i < 6; i++) {
					desired_pos_spin_boxes[i]->setValue(robot->desired_pos[i]);
				}
				init();
			}
		} // end if (robot->state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI_PTR
}

