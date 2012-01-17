#include "WgtToolBase.h"
#include "WgtToolBase.h"
#include "ui_robot.h"
#include "interface.h"

const int WgtToolBase::angle_axis_number = 6;

WgtToolBase::WgtToolBase(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent) :
		wgt_base(_widget_label, _interface, robo, parent)
{
	ui.setupUi(this);
	setup_ui(ui.gridLayout);
}

WgtToolBase::~WgtToolBase()
{

}

void WgtToolBase::setup_ui(QGridLayout *layout)
{
	wgt_base::setup_ui(layout, 6);
	wgt_base::create_spin_boxes(desired_pos_column, angle_axis_number);
	create_buttons_and_spin_boxes();
	create_buttons();
}

void WgtToolBase::create_buttons_and_spin_boxes()
{
	for (int i = 0; i < angle_axis_number; i++)
		add_current_position_spin_box(create_spin_box_to_vector(current_pos_spin_boxes), i);
}

void WgtToolBase::add_current_position_spin_box(QDoubleSpinBox *spin_box, int row)
{
	gridLayout->addWidget(spin_box, row + 1, 1, 1, 1);
}

void WgtToolBase::create_buttons()
{
	read_button = add_button("Read", 7, 1, 1, 1);
	set_button = add_button("Set", 7, 5, 1, 1);
	copy_button = add_button(">", 1, 3, angle_axis_number, 1);

	connect(read_button, SIGNAL(clicked()), this, SLOT(read_button_clicked()), Qt::QueuedConnection);
	connect(set_button, SIGNAL(clicked()), this, SLOT(execute_button_clicked()), Qt::QueuedConnection);
	connect(copy_button, SIGNAL(clicked()), this, SLOT(copy_button_clicked()), Qt::QueuedConnection);
}

void WgtToolBase::synchro_depended_widgets_disable(bool set_disabled)
{
	//set_button->setDisabled(set_disabled);
	copy_button->setDisabled(set_disabled);
	read_button->setDisabled(set_disabled);

	for (int i = 0; i < angle_axis_number; i++) {
		current_pos_spin_boxes[i]->setDisabled(set_disabled);
		desired_pos_spin_boxes[i]->setDisabled(set_disabled);
	}
}

void WgtToolBase::init_and_copy_slot()
{
	init();
	copy();
}

void WgtToolBase::read_button_clicked()
{
	printf("read\n");
	init();
}

void WgtToolBase::copy_button_clicked()
{
	copy();
}

void WgtToolBase::copy()
{
	if (robot->state.edp.pid != -1) {
		if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			set_button->setDisabled(false);
			for (int i = 0; i < angle_axis_number; i++) {
				desired_pos_spin_boxes[i]->setValue(current_pos_spin_boxes[i]->value());
			}
		} else
			set_button->setDisabled(true); // Wygaszanie elementow przy niezsynchronizowanym robocie
	}
}

void WgtToolBase::execute_button_clicked()
{
	get_desired_position();
	move_it();
}

void WgtToolBase::inc_move_left_button_clicked(int button)
{
	robot->zero_desired_position();
	robot->desired_pos[button] = -desired_pos_spin_boxes[button]->value();
	move_it();
}

void WgtToolBase::inc_move_right_button_clicked(int button)
{
	robot->zero_desired_position();
	robot->desired_pos[button] = desired_pos_spin_boxes[button]->value();
	move_it();
}
