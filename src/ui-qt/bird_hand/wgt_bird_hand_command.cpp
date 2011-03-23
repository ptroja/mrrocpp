#include "wgt_bird_hand_command.h"
#include "ui_wgt_bird_hand_command.h"
#include "ui_ecp_r_bird_hand.h"
#include "ui_r_bird_hand.h"
#include "../../robot/bird_hand/dp_bird_hand.h"
#include "../../robot/bird_hand/const_bird_hand.h"

//#include "ui/src/ui_ecp_r_single_motor.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"

#include "../base/interface.h"
#include "../base/mainwindow.h"

#include <QAbstractButton>
#include <QCheckBox>


wgt_bird_hand_command::wgt_bird_hand_command(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::bird_hand::UiRobot& _robot, QWidget *parent) :
    wgt_base("Bird hand incremental motion", _interface, parent),
    ui(new Ui::wgt_bird_hand_commandClass()),
    robot(_robot)

{
	//ui = new Ui::wgt_bird_hand_commandClass::wgt_bird_hand_commandClass();
    ui->setupUi(this);

	connect(this, SIGNAL(synchro_depended_init_signal()), this, SLOT(synchro_depended_init_slot()), Qt::QueuedConnection);
	connect(this, SIGNAL(init_and_copy_signal()), this, SLOT(init_and_copy_slot()), Qt::QueuedConnection);

    doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_thumb_0);
    doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_thumb_1);
    doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_index_0);
    doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_index_1);
    doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_index_2);
    doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_ring_0);
    doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_ring_1);
    doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_ring_2);

    doubleSpinBox_despos_Vector.append(ui->doubleSpinBox_despos_thumb_0);
    doubleSpinBox_despos_Vector.append(ui->doubleSpinBox_despos_thumb_1);
    doubleSpinBox_despos_Vector.append(ui->doubleSpinBox_despos_index_0);
    doubleSpinBox_despos_Vector.append(ui->doubleSpinBox_despos_index_1);
    doubleSpinBox_despos_Vector.append(ui->doubleSpinBox_despos_index_2);
    doubleSpinBox_despos_Vector.append(ui->doubleSpinBox_despos_ring_0);
    doubleSpinBox_despos_Vector.append(ui->doubleSpinBox_despos_ring_1);
    doubleSpinBox_despos_Vector.append(ui->doubleSpinBox_despos_ring_2);

    doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_thumb_0);
    doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_thumb_1);
    doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_index_0);
    doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_index_1);
    doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_index_2);
    doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_ring_0);
    doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_ring_1);
    doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_ring_2);

    doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_thumb_0);
    doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_thumb_1);
    doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_index_0);
    doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_index_1);
    doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_index_2);
    doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_ring_0);
    doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_ring_1);
    doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_ring_2);

    doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_thumb_0);
    doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_thumb_1);
    doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_index_0);
    doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_index_1);
    doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_index_2);
    doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_ring_0);
    doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_ring_1);
    doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_ring_2);

    doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_thumb_0);
    doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_thumb_1);
    doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_index_0);
    doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_index_1);
    doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_index_2);
    doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_ring_0);
    doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_ring_1);
    doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_ring_2);

    buttonGroup_Vector.append(ui->buttonGroup_thumb_0);
    buttonGroup_Vector.append(ui->buttonGroup_thumb_1);
    buttonGroup_Vector.append(ui->buttonGroup_index_0);
    buttonGroup_Vector.append(ui->buttonGroup_index_1);
    buttonGroup_Vector.append(ui->buttonGroup_index_2);
    buttonGroup_Vector.append(ui->buttonGroup_ring_0);
    buttonGroup_Vector.append(ui->buttonGroup_ring_1);
    buttonGroup_Vector.append(ui->buttonGroup_ring_2);

    checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_thumb_0);
    checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_thumb_1);
    checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_index_0);
    checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_index_1);
    checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_index_2);
    checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_ring_0);
    checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_ring_1);
    checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_ring_2);

    for(int i=0; i<robot.number_of_servos; i++)
    {
    	checkboxButtonGroup_Vector[i]->setExclusive(false);

    	doubleSpinBox_curtor_Vector[i]->setEnabled(false);
    	doubleSpinBox_mcur_Vector[i]->setEnabled(false);
    	doubleSpinBox_curpos_Vector[i]->setEnabled(false);

		QList<QAbstractButton*> buttons_in_group = checkboxButtonGroup_Vector[i]->buttons();
    	for(int j=0; j<buttons_in_group.size(); j++)
    	{
    		buttons_in_group[j]->setEnabled(false);
    	}
    }

}

wgt_bird_hand_command::~wgt_bird_hand_command()
{
  //  delete ui;
}


void wgt_bird_hand_command::my_open()
{
	wgt_base::my_open();
	init_and_copy_slot();
}

void wgt_bird_hand_command::synchro_depended_init()
{
	emit synchro_depended_init_signal();
}

void wgt_bird_hand_command::init_and_copy()
{
	emit init_and_copy_signal();
}

void wgt_bird_hand_command::on_pushButton_read_clicked()
{
	printf("read\n");
	init();
}

void wgt_bird_hand_command::init_and_copy_slot()
{
	init();
	copy_command();
}


void wgt_bird_hand_command::synchro_depended_init_slot()
{


}


void wgt_bird_hand_command::init()
{
	try {

		mrrocpp::lib::bird_hand::status &bhsrs = robot.ui_ecp_robot->bird_hand_status_reply_data_request_port->data;

	    joint_status.append(&bhsrs.thumb_f[0]);
	    joint_status.append(&bhsrs.thumb_f[1]);
	    joint_status.append(&bhsrs.index_f[0]);
	    joint_status.append(&bhsrs.index_f[1]);
	    joint_status.append(&bhsrs.index_f[2]);
	    joint_status.append(&bhsrs.ring_f[0]);
	    joint_status.append(&bhsrs.ring_f[1]);
	    joint_status.append(&bhsrs.ring_f[2]);

	    mrrocpp::lib::bird_hand::command &bhcs = robot.ui_ecp_robot->bird_hand_command_data_port->data;

	    joint_command.append(&bhcs.thumb_f[0]);
	    joint_command.append(&bhcs.thumb_f[1]);
	    joint_command.append(&bhcs.index_f[0]);
	    joint_command.append(&bhcs.index_f[1]);
	    joint_command.append(&bhcs.index_f[2]);
	    joint_command.append(&bhcs.ring_f[0]);
	    joint_command.append(&bhcs.ring_f[1]);
	    joint_command.append(&bhcs.ring_f[2]);

		if (robot.state.edp.pid != -1) {
			if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				synchro_depended_widgets_disable(false);

//				robot.ui_ecp_robot-> ;// co tutaj ma być?

				for (int i = 0; i < robot.number_of_servos; i++) {
					doubleSpinBox_curpos_Vector[i]->setValue(joint_status[i]->meassured_position);

				}

			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				synchro_depended_widgets_disable(true);

			}
		}

	} // end try
	CATCH_SECTION_UI


}

void wgt_bird_hand_command::on_pushButton_copy_clicked()
{
	copy_command();
}


int wgt_bird_hand_command::synchro_depended_widgets_disable(bool _set_disabled)
{
return 1;
}



int wgt_bird_hand_command::get_command()
{
	try {

		//lib::bird_hand::command &bhcs = robot.ui_ecp_robot->bird_hand_command_data_port->data;

		mrrocpp::lib::bird_hand::command &bhcs = robot.ui_ecp_robot->bird_hand_command_data_port->data;

		// odczyt ilosci krokow i ecp_query step

		bhcs.motion_steps = ui->spinBox_motion_steps->value();
		bhcs.ecp_query_step = ui->spinBox_query_step->value();

	    for(int i=0; i<robot.number_of_servos; i++)
	    {
	    	get_finger_command(i);
	    	get_variant_finger_command(i);
	    }



		//std::stringstream ss(std::stringstream::in | std::stringstream::out);
		/*
		 ss << bhcs.index_f[0].profile_type << " " << bhcs.motion_steps << "  "
		 << bhcs.ecp_query_step;
		 */
		/*
		 ss << bhcs.index_f[0].desired_position << " "
		 << bhcs.index_f[0].desired_torque << "  "
		 << bhcs.index_f[0].reciprocal_of_damping;

		 interface.ui_msg->message(ss.str().c_str());
		 */
		robot.ui_ecp_robot->bird_hand_command_data_port->set();
		robot.ui_ecp_robot->execute_motion();

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int wgt_bird_hand_command::set_status()
{
	try
	{
		if (robot.state.edp.pid != -1)
		{
			robot.ui_ecp_robot->bird_hand_status_reply_data_request_port->set_request();
			robot.ui_ecp_robot->execute_motion();
			robot.ui_ecp_robot->bird_hand_status_reply_data_request_port->get();

			if (robot.state.edp.is_synchronised)
						for(int i=0; i<robot.number_of_servos; i++)
						{
							set_finger_status(i);
						}
			init();
		}
	} // end try
	CATCH_SECTION_UI

	return 1;
}

int wgt_bird_hand_command::copy_command()
{
	if (robot.state.edp.pid != -1) {
		if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			ui->pushButton_execute->setDisabled(false);

			for(int i=0; i<robot.number_of_servos; i++)
			{
				get_variant_finger_command(i);
				copy_finger_command(i);
			}

		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			ui->pushButton_execute->setDisabled(true);
		}

	}

	return 1;
}

void wgt_bird_hand_command::on_pushButton_execute_clicked()
{
	get_desired_position();
	set_status();
}

int wgt_bird_hand_command::get_desired_position()
{

	if (robot.state.edp.pid != -1) {

		if (robot.state.edp.is_synchronised) {

			for (int i = 0; i < robot.number_of_servos; i++) {
			//	robot.desired_pos[i] = doubleSpinBox_despos_Vector[i]->value(); //co tu ma być?
				joint_command[i]->desired_position = doubleSpinBox_despos_Vector[i]->value();
			}
		} else {

			for (int i = 0; i < robot.number_of_servos; i++) {
				joint_command[i]->desired_position = 0.0;
			}
		}
	}
	return 1;
}


int wgt_bird_hand_command::get_variant_finger_command(int fingerId)
{
    	QList<QAbstractButton*> buttons_in_group= buttonGroup_Vector[fingerId]->buttons();

    	for(int i=0; i<buttons_in_group.size(); i++)
    	{
    		 if(buttons_in_group[i]->isChecked())
    		 {
    			 switch (i)
    			 {
    				 case 0:
    					 joint_command[fingerId]->profile_type = lib::bird_hand::MACROSTEP_ABSOLUTE_POSITION;
    					 break;
    				 case 1:
    					 joint_command[fingerId]->profile_type = lib::bird_hand::MACROSTEP_POSITION_INCREMENT;
    					 break;
    				 case 2:
    					 joint_command[fingerId]->profile_type = lib::bird_hand::SIGLE_STEP_POSTION_INCREMENT;
    					 break;
    				 default:
    					 break;
    			 }
    			 return 1;
    		 }
    	}
    return 1;
}

int wgt_bird_hand_command::get_finger_command(int fingerId)
{
	joint_command[fingerId]->desired_position = doubleSpinBox_despos_Vector[fingerId]->value();
	joint_command[fingerId]->desired_torque = doubleSpinBox_destor_Vector[fingerId]->value();
	joint_command[fingerId]->reciprocal_of_damping = doubleSpinBox_rdamp_Vector[fingerId]->value();

	return 1;
}

int wgt_bird_hand_command::set_finger_status(int fingerId)
{
	QList<QAbstractButton*> chboxes = checkboxButtonGroup_Vector[fingerId]->buttons();

	doubleSpinBox_curpos_Vector[fingerId]->setValue(joint_status[fingerId]->meassured_position);
	doubleSpinBox_curtor_Vector[fingerId]->setValue(joint_status[fingerId]->meassured_torque);
	doubleSpinBox_mcur_Vector[fingerId]->setValue(joint_status[fingerId]->meassured_current);

	if (joint_status[fingerId]->lower_limit_of_absolute_position) {
		chboxes[0]->setChecked(true);
	} else {
		chboxes[0]->setChecked(false);
	}

	if (joint_status[fingerId]->lower_limit_of_absolute_value_of_desired_torque) {
		chboxes[1]->setChecked(true);
	} else {
		chboxes[1]->setChecked(false);
	}

	if (joint_status[fingerId]->upper_limit_of_absolute_position) {
		chboxes[2]->setChecked(true);
	} else {
		chboxes[2]->setChecked(false);
	}

	if (joint_status[fingerId]->upper_limit_of_absolute_value_of_computed_position_increment) {
		chboxes[3]->setChecked(true);
	} else {
		chboxes[3]->setChecked(false);
	}

	if (joint_status[fingerId]->upper_limit_of_absolute_value_of_desired_position_increment) {
		chboxes[4]->setChecked(true);
	} else {
		chboxes[4]->setChecked(false);
	}

	if (joint_status[fingerId]->upper_limit_of_absolute_value_of_desired_torque) {
		chboxes[5]->setChecked(true);
	} else {
		chboxes[5]->setChecked(false);
	}

	if (joint_status[fingerId]->upper_limit_of_absolute_value_of_meassured_torque) {
		chboxes[6]->setChecked(true);
	} else {
		chboxes[6]->setChecked(false);
	}

	if (joint_status[fingerId]->upper_limit_of_meassured_current) {
		chboxes[7]->setChecked(true);
	} else {
		chboxes[7]->setChecked(false);
	}

	return 1;
}

int wgt_bird_hand_command::copy_finger_command(int fingerId)
{

	if (joint_command[fingerId]->profile_type == lib::bird_hand::MACROSTEP_ABSOLUTE_POSITION)
		doubleSpinBox_despos_Vector[fingerId]->setValue(doubleSpinBox_curpos_Vector[fingerId]->value());

	return 1;
}

