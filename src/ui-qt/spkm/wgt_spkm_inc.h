#ifndef WGT_SPKM_INC_H
#define WGT_SPKM_INC_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_spkm_inc.h"
#include "../base/wgt_base.h"
#include <QTimer>

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class UiRobot;
}
namespace spkm {
class UiRobot;
const std::string WGT_SPKM_INC = "WGT_SPKM_INC";
}
}
}

class wgt_spkm_inc : public wgt_base
{
Q_OBJECT

public:
	wgt_spkm_inc(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent =
			0);
	~wgt_spkm_inc();

	QVector <QDoubleSpinBox*> doubleSpinBox_cur_Vector;
	QVector <QDoubleSpinBox*> doubleSpinBox_mcur_Vector;
	QVector <QDoubleSpinBox*> doubleSpinBox_des_Vector;
	QVector <QRadioButton*> radioButton_mip_Vector;

	void synchro_depended_init();

private:
	Ui::wgt_spkm_incClass ui;
	mrrocpp::ui::spkm::UiRobot* robot;

	int init();
	int copy();

	int synchro_depended_widgets_disable(bool _set_disabled);

	int set_single_axis(int axis, QDoubleSpinBox* qdsb_mcur, QDoubleSpinBox* qdsb_cur_p, QAbstractButton* qab_mip);
	int get_desired_position();
	int move_it();

	QTimer *timer;

	void showEvent(QShowEvent * event);

signals:
	void synchro_depended_init_signal();

private slots:

	void synchro_depended_init_slot();

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

#endif // WGT_SPKM_INC_H
