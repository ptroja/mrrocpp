/*
 * WgtRelativeBase.cpp
 *
 *  Created on: 14-07-2011
 *      Author: matt
 */

#include "WgtRelativeBase.h"
#include "ui_robot.h"
#include "interface.h"

const int WgtRelativeBase::angle_axis_number = 6;

WgtRelativeBase::WgtRelativeBase(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent) :
		wgt_base(_widget_label, _interface, robo, parent)
{

}

WgtRelativeBase::~WgtRelativeBase()
{

}

void WgtRelativeBase::setup_ui(QGridLayout *layout)
{
	wgt_base::setup_ui(layout, 6);
	wgt_base::create_buttons_and_spin_boxes(desired_pos_column, inc_move_column, angle_axis_number);

	create_buttons();
}

void WgtRelativeBase::create_buttons()
{
	l_button = add_button("<", 1, 7, 6, 1);
	r_button = add_button(">", 1, 11, 6, 1);

	connect(l_button, SIGNAL(clicked()), this, SLOT(l_button_clicked()), Qt::QueuedConnection);
	connect(r_button, SIGNAL(clicked()), this, SLOT(r_button_clicked()), Qt::QueuedConnection);
}

void WgtRelativeBase::synchro_depended_widgets_disable(bool set_disabled)
{
	for (int i = 0; i < angle_axis_number; i++) {
		desired_pos_spin_boxes[i]->setDisabled(set_disabled);
	}
}

void WgtRelativeBase::init_and_copy_slot()
{
	init();
	//copy();
}

void WgtRelativeBase::r_button_clicked()
{
	robot->zero_desired_position();
	for (int i = 0; i < angle_axis_number; i++)
		robot->desired_pos[i] = desired_pos_spin_boxes[i]->value();

	move_it();
}

void WgtRelativeBase::l_button_clicked()
{
	robot->zero_desired_position();
	for (int i = 0; i < angle_axis_number; i++)
		robot->desired_pos[i] = -desired_pos_spin_boxes[i]->value();

	move_it();
}

void WgtRelativeBase::inc_move_left_button_clicked(int button)
{
	robot->zero_desired_position();
	robot->desired_pos[button] = -desired_pos_spin_boxes[button]->value();
	move_it();
}

void WgtRelativeBase::inc_move_right_button_clicked(int button)
{
	robot->zero_desired_position();
	robot->desired_pos[button] = desired_pos_spin_boxes[button]->value();
	move_it();
}
