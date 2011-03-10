#ifndef WGT_IRP6OT_M_JOINTS_H
#define WGT_IRP6OT_M_JOINTS_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_irp6ot_m_joints.h"
#include "../base/wgt_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace irp6ot_m {
class UiRobot;
const std::string WGT_IRP6OT_JOINTS = "WGT_IRP6OT_JOINTS";
}
}
}


class wgt_irp6ot_m_joints : public wgt_base
{
Q_OBJECT

public:
	wgt_irp6ot_m_joints(mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::irp6ot_m::UiRobot& _robot, QWidget *parent = 0);
	~wgt_irp6ot_m_joints();
	void my_open();

private:
	Ui::wgt_irp6ot_m_jointsClass ui;
	mrrocpp::ui::irp6ot_m::UiRobot& robot;

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
