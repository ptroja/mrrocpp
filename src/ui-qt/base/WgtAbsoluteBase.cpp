/*
 * WgtAbsoluteBase.cpp
 *
 *  Created on: 14-07-2011
 *      Author: matt
 */

#include "WgtAbsoluteBase.h"
#include "ui_robot.h"
#include "interface.h"
#include "mainwindow.h"

WgtAbsoluteBase::WgtAbsoluteBase(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent) :
		wgt_base(_widget_label, _interface, robo, parent)
{

}

WgtAbsoluteBase::~WgtAbsoluteBase()
{
	// TODO Auto-generated destructor stub
}

void WgtAbsoluteBase::setup_ui(QGridLayout *layout, int _rows_number)
{
	wgt_base::setup_ui(layout, _rows_number);

	create_buttons_and_spin_boxes();
	create_buttons();
	create_step_spinbox();

	wgt_base::create_buttons_and_spin_boxes(desired_pos_column, inc_move_column, rows_number);
}

void WgtAbsoluteBase::create_step_spinbox()
{
	step_spinbox = new QDoubleSpinBox(this);
	step_spinbox->setDecimals(3);
	step_spinbox->setMaximum(10);
	step_spinbox->setSingleStep(0.01);

	gridLayout->addWidget(step_spinbox, 8, 10, 1, 2);
}

void WgtAbsoluteBase::create_buttons_and_spin_boxes()
{
	for (int i = 0; i < rows_number; i++)
		add_current_position_spin_box(create_spin_box_to_vector(current_pos_spin_boxes), i);
}

void WgtAbsoluteBase::add_current_position_spin_box(QDoubleSpinBox *spin_box, int row)
{
	gridLayout->addWidget(spin_box, row + 1, 1, 1, 1);
}

void WgtAbsoluteBase::create_buttons()
{
	read_button = add_button("Read", 8, 1, 1, 1);
	execute_button = add_button("Move", 8, 6, 1, 3);
	import_button = add_button("Import", 9, 6, 1, 1);
	export_button = add_button("Export", 9, 7, 1, 1);
	copy_button = add_button(">", 1, 3, rows_number, 1);

	connect(read_button, SIGNAL(clicked()), this, SLOT(read_button_clicked()), Qt::QueuedConnection);
	connect(execute_button, SIGNAL(clicked()), this, SLOT(execute_button_clicked()), Qt::QueuedConnection);
	connect(import_button, SIGNAL(clicked()), this, SLOT(import_button_clicked()), Qt::QueuedConnection);
	connect(export_button, SIGNAL(clicked()), this, SLOT(export_button_clicked()), Qt::QueuedConnection);
	connect(copy_button, SIGNAL(clicked()), this, SLOT(copy_button_clicked()), Qt::QueuedConnection);
}

void WgtAbsoluteBase::synchro_depended_widgets_disable(bool set_disabled)
{
	execute_button->setDisabled(set_disabled);
	copy_button->setDisabled(set_disabled);
	export_button->setDisabled(set_disabled);
	import_button->setDisabled(set_disabled);
	read_button->setDisabled(set_disabled);

	for (int i = 0; i < rows_number; i++) {
		current_pos_spin_boxes[i]->setDisabled(set_disabled);
		desired_pos_spin_boxes[i]->setDisabled(set_disabled);
	}

}

void WgtAbsoluteBase::inc_move_left_button_clicked(int button)
{
	get_desired_position();
	robot->desired_pos[button] -= step_spinbox->value();
	move_it();
}

void WgtAbsoluteBase::inc_move_right_button_clicked(int button)
{
	get_desired_position();
	robot->desired_pos[button] += step_spinbox->value();
	move_it();
}

void WgtAbsoluteBase::read_button_clicked()
{
	printf("read\n");
	init();
}

void WgtAbsoluteBase::import_button_clicked()
{
	double val[rows_number];

	for (int i = 0; i < rows_number; i++)
		val[i] = 0.0;

	interface.get_main_window()->get_lineEdit_position(val, rows_number);

	for (int i = 0; i < rows_number; i++)
		desired_pos_spin_boxes[i]->setValue(val[i]);
}

void WgtAbsoluteBase::export_button_clicked()
{
	std::stringstream buffer(std::stringstream::in | std::stringstream::out);

	buffer << widget_label.toStdString() << " INCREMENTAL POSITION\n ";
	for (int i = 0; i < rows_number; i++)
		buffer << " " << desired_pos_spin_boxes[i]->value();

	interface.ui_msg->message(buffer.str());
}

void WgtAbsoluteBase::init_and_copy_slot()
{
	init();
	copy();
}

void WgtAbsoluteBase::copy_button_clicked()
{
	copy();
}

int WgtAbsoluteBase::copy()
{

	if (robot->state.edp.pid != -1) {
		if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			execute_button->setDisabled(false);
			for (int i = 0; i < rows_number; i++) {
				desired_pos_spin_boxes[i]->setValue(current_pos_spin_boxes[i]->value());
			}
		} else
			execute_button->setDisabled(true); // Wygaszanie elementow przy niezsynchronizowanym robocie
	}

	return 1;
}

void WgtAbsoluteBase::execute_button_clicked()
{
	get_desired_position();
	move_it();
}

