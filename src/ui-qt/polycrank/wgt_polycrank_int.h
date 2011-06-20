#ifndef WGT_POLYCRANK_INT_H
#define WGT_POLYCRANK_INT_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include <QVector>
#include "ui_wgt_polycrank_int.h"
#include "../base/wgt_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace polycrank {
class UiRobot;
const std::string WGT_POLYCRANK_INT = "WGT_POLYCRANK_INT";
}
}
}

class wgt_polycrank_int : public wgt_base
{
Q_OBJECT

public:
			wgt_polycrank_int(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::polycrank::UiRobot& _robot, QWidget *parent =
					0);
	~wgt_polycrank_int();

	QVector <QDoubleSpinBox*> doubleSpinBox_cur_Vector;
	QVector <QDoubleSpinBox*> doubleSpinBox_des_Vector;

private:
	Ui::wgt_polycrank_intClass ui;
	mrrocpp::ui::polycrank::UiRobot& robot;

	int init();
	int copy();
	int get_desired_position();
	int move_it();
	//int motion(/* TR PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo*/);
	//int set_single_axis(int axis, QDoubleSpinBox* qdsb_mcur, QDoubleSpinBox* qdsb_cur_p, QAbstractButton* qab_mip);

private slots:

	void on_pushButton_read_clicked();
	void on_pushButton_export_clicked();
	void on_pushButton_import_clicked();
	void on_pushButton_copy_clicked();

	void on_pushButton_execute_clicked();
	void on_pushButton_1l_clicked();
	void on_pushButton_2l_clicked();
	void on_pushButton_3l_clicked();
	void on_pushButton_4l_clicked();
	void on_pushButton_5l_clicked();
	void on_pushButton_6l_clicked();
	void on_pushButton_7l_clicked();
	void on_pushButton_1r_clicked();
	void on_pushButton_2r_clicked();
	void on_pushButton_3r_clicked();
	void on_pushButton_4r_clicked();
	void on_pushButton_5r_clicked();
	void on_pushButton_6r_clicked();
	void on_pushButton_7r_clicked();
};

#endif // WGT_SPKM_INC_H
