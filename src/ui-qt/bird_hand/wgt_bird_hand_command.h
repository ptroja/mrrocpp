#ifndef WGT_BIRD_HAND_COMMAND_H
#define WGT_BIRD_HAND_COMMAND_H

#include <QWidget>
#include "../base/wgt_base.h"
#include "robot/bird_hand/const_bird_hand.h"
#include "../base/ui_robot.h"
#include "ui_wgt_bird_hand_command.h"
#include <QVector>
#include <QDoubleSpinBox>
#include <QButtonGroup>

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class UiRobot;
}
namespace bird_hand {
class UiRobot;
const std::string WGT_BIRD_HAND_COMMAND = "WGT_BIRD_HAND_COMMAND";
}
}
}

namespace Ui {
class wgt_bird_hand_commandClass;
}

class wgt_bird_hand_command : public wgt_base
{
Q_OBJECT

public:
	explicit wgt_bird_hand_command(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent =
			0);
	~wgt_bird_hand_command();

	int get_command();
	int set_status();
	int copy_command();

	int get_variant_finger_command(int fingerId);
	int get_finger_command(int fingerId);
	int set_finger_status(int fingerId);

	int copy_finger_command(int fingerId);

	void my_open(bool set_on_top = false);
	virtual void add_button(QWidget *button, int row, int space)
	{
	}

	virtual void setup_ui()
	{
	}

	QVector <QDoubleSpinBox*> doubleSpinBox_curtor_Vector;
	QVector <QDoubleSpinBox*> doubleSpinBox_destor_Vector;
	QVector <QDoubleSpinBox*> doubleSpinBox_rdamp_Vector;
	QVector <QDoubleSpinBox*> doubleSpinBox_mcur_Vector;
	QVector <QDoubleSpinBox*> doubleSpinBox_curpos_Vector;
	QVector <QDoubleSpinBox*> desired_pos_spin_box;

	QVector <QButtonGroup*> buttonGroup_Vector;
	QVector <QButtonGroup*> checkboxButtonGroup_Vector;

private:
	Ui::wgt_bird_hand_commandClass *ui;
	mrrocpp::ui::bird_hand::UiRobot* robot;

	QVector <lib::bird_hand::single_joint_command*> joint_command;
	QVector <lib::bird_hand::single_joint_status*> joint_status;

	void synchro_depended_init();
	void init_and_copy();
	void init();
	int synchro_depended_widgets_disable(bool _set_disabled);
	int get_desired_position();

	lib::bird_hand::MOTION_VARIANT current_profile_type;

signals:
	void synchro_depended_init_signal();
	void init_and_copy_signal();

private slots:
	void on_pushButton_read_clicked();
	void on_pushButton_execute_clicked();
	void on_pushButton_copy_clicked();
	void on_pushButton_clear_all_clicked();
	void on_pushButton_change_command_type_all_clicked();

	void init_and_copy_slot();
	void synchro_depended_init_slot();

};

#endif // WGT_BIRD_HAND_COMMAND_H
