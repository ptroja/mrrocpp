#ifndef WGT_IRP6P_TFG_INC_H
#define WGT_IRP6P_TFG_INC_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_irp6p_tfg_move.h"
#include "../base/wgt_base.h"
#include <QTimer>

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace irp6p_tfg {
class UiRobot;
const std::string WGT_IRP6P_TFG_MOVE = "WGT_IRP6P_TFG_MOVE";
}
}
}

class wgt_irp6p_tfg_move : public wgt_base
{
Q_OBJECT

public:
			wgt_irp6p_tfg_move(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::irp6p_tfg::UiRobot& _robot, QWidget *parent =
					0);
	~wgt_irp6p_tfg_move();

	void synchro_depended_init();

private:
	Ui::wgt_irp6p_tfg_moveClass ui;
	mrrocpp::ui::irp6p_tfg::UiRobot& robot;

	QTimer *timer;

	int synchro_depended_widgets_disable(bool _set_disabled);

	int init_mr();
	int copy_mr();

	int get_desired_position_mr();
	int move_it_mr();

signals:
	void synchro_depended_init_signal();

private slots:

	void synchro_depended_init_slot();

	void on_pushButton_read_mr_clicked();
	void on_pushButton_export_mr_clicked();
	void on_pushButton_import_mr_clicked();
	void on_pushButton_copy_mr_clicked();

	void on_pushButton_execute_mr_clicked();
	void on_pushButton_l_mr_clicked();
	void on_pushButton_r_mr_clicked();

};

#endif // WGT_IRP6P_TFG_INC_H
