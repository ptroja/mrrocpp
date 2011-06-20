#ifndef WGT_SPKM_EXT_H
#define WGT_SPKM_EXT_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_spkm_ext.h"
#include "../base/wgt_base.h"
#include <QTimer>

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace spkm {
class UiRobot;
const std::string WGT_SPKM_EXT = "WGT_SPKM_EXT";
}
}
}

class wgt_spkm_ext : public wgt_base
{
Q_OBJECT

public:
	wgt_spkm_ext(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::spkm::UiRobot& _robot, QWidget *parent = 0);
	~wgt_spkm_ext();

	QVector <QDoubleSpinBox*> doubleSpinBox_cur_Vector;
	QVector <QDoubleSpinBox*> doubleSpinBox_mcur_Vector;
	QVector <QDoubleSpinBox*> doubleSpinBox_des_Vector;
	QVector <QRadioButton*> radioButton_mip_Vector;

private:
	Ui::wgt_spkm_extClass ui;
	mrrocpp::ui::spkm::UiRobot& robot;

	int init();
	int copy();

	int set_single_axis(int axis, QDoubleSpinBox* qdsb_mcur, QAbstractButton* qab_mip);
	int get_desired_position();
	int move_it();
	QTimer *timer;

private slots:
	void timer_slot();
	void on_pushButton_read_clicked();
	void on_pushButton_export_clicked();
	void on_pushButton_import_clicked();
	void on_pushButton_copy_clicked();
	void on_pushButton_stop_clicked();

	void on_pushButton_execute_clicked();
	void on_pushButton_0l_clicked();
	void on_pushButton_1l_clicked();
	void on_pushButton_2l_clicked();
	void on_pushButton_3l_clicked();
	void on_pushButton_4l_clicked();
	void on_pushButton_5l_clicked();
	void on_pushButton_0r_clicked();
	void on_pushButton_1r_clicked();
	void on_pushButton_2r_clicked();
	void on_pushButton_3r_clicked();
	void on_pushButton_4r_clicked();
	void on_pushButton_5r_clicked();

};

#endif // WGT_SPKM_EXT_H
