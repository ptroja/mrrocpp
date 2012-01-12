//#include "ui_ecp_r_irp6_m.h"
#include "ui_r_irp6_m.h"

//#include "ui/src/ui_ecp_r_single_motor.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common012.h"
#include "wgt_irp6_m_motors.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"
#include <iostream>

wgt_irp6_m_motors::wgt_irp6_m_motors(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent) :
		WgtAbsoluteBase(_widget_label, _interface, _robot, parent)
{
	ui.setupUi(this);
	specyficrobot = dynamic_cast <mrrocpp::ui::irp6_m::UiRobot *>(_robot);

//	if (robot->robot_name == lib::irp6ot_m::ROBOT_NAME)
//	{
//	//	current_pos_spin_boxes.append(ui.doubleSpinBox_cur_p7);
//		//desired_pos_spin_boxes.append(ui.doubleSpinBox_des_p7);
//	}
	if (robot->robot_name == lib::irp6p_m::ROBOT_NAME) {
		ui.label_axis_7->hide();
		//ui.doubleSpinBox_cur_p7->hide();
		/*		ui.doubleSpinBox_des_p7->hide();*/
	}

	setup_ui(ui.gridLayout, robot->number_of_servos);
}

void wgt_irp6_m_motors::setup_ui(QGridLayout *layout, int _rows_number)
{
	WgtAbsoluteBase::setup_ui(layout, _rows_number);

}

wgt_irp6_m_motors::~wgt_irp6_m_motors()
{

}

void wgt_irp6_m_motors::init()
{
	try {
		if (robot->state.edp.pid != -1) {
			if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				synchro_depended_widgets_disable(false);
				specyficrobot->ui_ecp_robot->read_motors(robot->current_pos);

				printf("\t\t init ok\n");

				for (int i = 0; i < robot->number_of_servos; i++) {
					current_pos_spin_boxes[i]->setValue(robot->current_pos[i]);
					robot->desired_pos[i] = robot->current_pos[i];
				}
			} else
				synchro_depended_widgets_disable(true);
		}
	}
	CATCH_SECTION_UI_PTR
}

void wgt_irp6_m_motors::move_it()
{
	// wychwytania ew. bledow ECP::robot
	try {

		if (robot->state.edp.pid != -1) {

			//robot->ui_ecp_robot->move_motors(robot->desired_pos);

			//robot->ui_ecp_robot->move_motors(robot->desired_pos);
			specyficrobot->ui_ecp_robot->move_motors(robot->desired_pos);

			//robot->ui_ecp_robot->interface.irp6_m->ui_ecp_robot->move_motors(robot->desired_pos);
			//interface.irp6_m->ui_ecp_robot->move_motors(interface.irp6_m->desired_pos);
			//interface.irp6_m->ui_ecp_robot->move_motors(interface.irp6_m->desired_pos);
			//robot->ui_ecp_robot->move_motors(robot->desired_pos);

			if ((robot->state.edp.is_synchronised) /* TR && (is_open)*/) { // by Y o dziwo nie dziala poprawnie 	 if (robot->state.edp.is_synchronised)
				printf("\t\t move it ok\n");
				for (int i = 0; i < robot->number_of_servos; i++) {
					desired_pos_spin_boxes[i]->setValue(robot->desired_pos[i]);
				}
				init();
			}
		} // end if (robot->state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI_PTR
}

