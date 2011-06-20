#ifndef WGT_IRP6_M_TOOL_EULER_H
#define WGT_IRP6_M_TOOL_EULER_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include <QVector>
#include "ui_wgt_irp6_m_tool_euler.h"
#include "../base/wgt_base.h"

#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

#include "../../base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace irp6_m {
class UiRobot;

}
}
}

namespace mrrocpp {
namespace lib {
class Xyz_Euler_Zyz_vector;
}
}

class wgt_irp6_m_tool_euler : public wgt_base
{
Q_OBJECT

public:
			wgt_irp6_m_tool_euler(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::irp6_m::UiRobot& _robot, QWidget *parent =
					0);
	~wgt_irp6_m_tool_euler();
	void synchro_depended_init();
	void init_and_copy();
	void my_open(bool set_on_top=false);
	QString robot_label;

	QVector <QDoubleSpinBox*> doubleSpinBox_cur_Vector;
	QVector <QDoubleSpinBox*> doubleSpinBox_des_Vector;

private:
	Ui::wgt_irp6_m_tool_eulerClass ui;
	mrrocpp::ui::irp6_m::UiRobot& robot;

	int init();
	int copy();

	int synchro_depended_widgets_disable(bool _set_disabled);

	int get_desired_position();
	int move_it();
	//int motion(/* TR PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo*/);
	//int set_single_axis(int axis, QDoubleSpinBox* qdsb_mcur, QDoubleSpinBox* qdsb_cur_p, QAbstractButton* qab_mip);
	const static int aa_number;
	mrrocpp::lib::Xyz_Euler_Zyz_vector tool_vector;

signals:
	void synchro_depended_init_signal();
	void init_and_copy_signal();

private slots:

	void synchro_depended_init_slot();
	void init_and_copy_slot();

	void on_pushButton_read_clicked();
	void on_pushButton_copy_clicked();

	void on_pushButton_set_clicked();

};

#endif // WGT_SPKM_INC_H
