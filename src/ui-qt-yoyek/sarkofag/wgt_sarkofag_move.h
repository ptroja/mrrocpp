#ifndef WGT_SARKOFAG_INC_H
#define WGT_SARKOFAG_INC_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_sarkofag_move.h"
#include "../base/wgt_base.h"
#include <QTimer>

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace sarkofag {
class UiRobot;
const std::string WGT_SARKOFAG_MOVE = "WGT_SARKOFAG_MOVE";
}
}
}

class wgt_sarkofag_move : public wgt_base
{
Q_OBJECT

public:
			wgt_sarkofag_move(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::sarkofag::UiRobot& _robot, QWidget *parent =
					0);
	~wgt_sarkofag_move();

	void synchro_depended_init();
	void my_open();

private:
	Ui::wgt_sarkofag_moveClass ui;
	mrrocpp::ui::sarkofag::UiRobot& robot;

	QTimer *timer;

	int synchro_depended_widgets_disable(bool _set_disabled);

	int init_mr();
	int copy_mr();

	int get_desired_position_mr();
	int move_it_mr();

	int init_si();
	int copy_si();

	int get_desired_position_si();
	int move_it_si();

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

	void on_pushButton_read_si_clicked();
	void on_pushButton_export_si_clicked();
	void on_pushButton_import_si_clicked();
	void on_pushButton_copy_si_clicked();

	void on_pushButton_execute_si_clicked();
	void on_pushButton_l_si_clicked();
	void on_pushButton_r_si_clicked();

};

#endif // WGT_SARKOFAG_INC_H
