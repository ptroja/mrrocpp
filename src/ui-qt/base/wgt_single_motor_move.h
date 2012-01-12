#ifndef WGT_SINGLE_MOTOR_MOVE_H
#define WGT_SINGLE_MOTOR_MOVE_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_single_motor_move.h"
#include "../base/wgt_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class UiRobot;
}
namespace single_motor {
class UiRobot;

}
}
}

class wgt_single_motor_move : public wgt_base
{
	Q_OBJECT

public:
	wgt_single_motor_move(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot* _robot, QWidget *parent =
			0);
	~wgt_single_motor_move();

	void synchro_depended_init();
	void init_and_copy();
	void my_open(bool set_on_top = false);
	virtual void add_button(QPushButton *button, int row, int space)
	{
	}
	;
	virtual void setup_ui()
	{
	}
	;

private:
	Ui::wgt_single_motor_moveClass ui;
	mrrocpp::ui::single_motor::UiRobot* robot;

	void init_mr_and_si();
	void copy_mr_and_si();

	void synchro_depended_widgets_disable(bool _set_disabled);

	void init_mr();
	void copy_mr();

	void get_desired_position_mr();
	void move_it_mr();

	void init_si();
	void copy_si();

	void get_desired_position_si();
	void move_it_si();

	signals:
	void synchro_depended_init_signal();
	void init_and_copy_signal();

private slots:

	void synchro_depended_init_slot();
	void init_and_copy_slot();

	void on_pushButton_read_mr_clicked();
	void on_pushButton_export_mr_clicked();
	void on_pushButton_import_mr_clicked();
	void on_pushButton_copy_mr_clicked();

	void on_pushButton_execute_mr_clicked();
	void on_pushButton_l_mr_clicked();
	void on_pushButton_r_mr_clicked();

	void on_pushButton_read_si_clicked();
	void on_pushButton_export_si_clicked();
	void on_pushButton_import_si_clicked();
	void on_pushButton_copy_si_clicked();

	void on_pushButton_execute_si_clicked();
	void on_pushButton_l_si_clicked();
	void on_pushButton_r_si_clicked();

};

#endif // WGT_CONVEYOR_INC_H
