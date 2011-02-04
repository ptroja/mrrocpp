#ifndef WGT_IRP6P_TFG_INC_H
#define WGT_IRP6P_TFG_INC_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_irp6p_tfg_inc.h"
#include "../base/wgt_base.h"
#include <QTimer>

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace irp6p_tfg {
class UiRobot;
const std::string WGT_IRP6P_TFG_INC = "WGT_IRP6P_TFG_INC";
}
}
}

class wgt_irp6p_tfg_inc : public wgt_base
{
Q_OBJECT

public:
			wgt_irp6p_tfg_inc(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::irp6p_tfg::UiRobot& _robot, QWidget *parent =
					0);
	~wgt_irp6p_tfg_inc();

	void synchro_depended_init();

private:
	Ui::wgt_irp6p_tfg_incClass ui;
	mrrocpp::ui::irp6p_tfg::UiRobot& robot;

	int init();
	int copy();

	int synchro_depended_widgets_disable(bool _set_disabled);

	int set_single_axis(int axis, QDoubleSpinBox* qdsb_mcur, QDoubleSpinBox* qdsb_cur_p, QAbstractButton* qab_mip);
	int get_desired_position();
	int move_it();

	QTimer *timer;

signals:
	void synchro_depended_init_signal();

private slots:

	void synchro_depended_init_slot();

	void on_timer_slot();
	void on_pushButton_read_clicked();
	void on_pushButton_export_clicked();
	void on_pushButton_import_clicked();
	void on_pushButton_copy_clicked();
	void on_pushButton_stop_clicked();

	void on_pushButton_execute_clicked();
	void on_pushButton_0l_clicked();
	void on_pushButton_0r_clicked();

};

#endif // WGT_IRP6P_TFG_INC_H
