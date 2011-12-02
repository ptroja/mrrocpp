#ifndef WGT_SHEAD_COMMAND_H
#define WGT_SHEAD_COMMAND_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_shead_command.h"
#include "../base/wgt_base.h"
#include <QTimer>

#include <boost/shared_ptr.hpp>

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class UiRobot;
}
namespace shead {
class UiRobot;
const std::string WGT_SHEAD_COMMAND = "WGT_SHEAD_COMMAND";
}
}
}

class wgt_shead_command : public wgt_base
{
Q_OBJECT

public:
	wgt_shead_command(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent =
			0);
	~wgt_shead_command();

	QVector <QCheckBox*> checkBox_m_mip_Vector;
	QVector <QCheckBox*> checkBox_contacts_Vector;

	QVector <QDoubleSpinBox*> doubleSpinBox_m_current_position_Vector;
	QVector <QDoubleSpinBox*> doubleSpinBox_m_absolute_Vector;
	QVector <QDoubleSpinBox*> doubleSpinBox_m_relative_Vector;

	void synchro_depended_init();

	void showEvent(QShowEvent * event);

private:
	Ui::wgt_shead_commandClass ui;
	mrrocpp::ui::shead::UiRobot* robot;

	int init();

	int synchro_depended_widgets_disable(bool _set_disabled);

	int get_desired_position();
	int move_it();

	boost::shared_ptr <QTimer> timer;

signals:
	void synchro_depended_init_signal();

private slots:

	void synchro_depended_init_slot();

	void timer_slot();

	// buttons callbacks

//	void on_pushButton_fl_execute_clicked();
//	void on_pushButton_fl_all_up_clicked();
//	void on_pushButton_fl_all_down_clicked();

	void on_pushButton_sol_execute_clicked();
	void on_pushButton_vac_execute_clicked();

	void on_pushButton_m_execute_clicked();
	void on_pushButton_stop_clicked();

	void on_pushButton_read_clicked();
	void on_pushButton_ml_copy_clicked();
	void on_pushButton_ml_left_clicked();
	void on_pushButton_ml_rigth_clicked();

	void on_radioButton_m_motor_toggled();
	void on_radioButton_m_joint_toggled();
	void on_radioButton_m_ext_toggled();

};

#endif // WGT_SHEAD_COMMAND_H
