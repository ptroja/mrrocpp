#ifndef WGT_IRP6_M_EULER_H
#define WGT_IRP6_M_EULER_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include <QVector>
#include "ui_wgt_absolute_template.h"
#include "../base/WgtAbsoluteBase.h"

#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class UiRobot;
}
namespace irp6_m {
class UiRobot;

}
}
}

class wgt_irp6_m_euler : public WgtAbsoluteBase
{
	Q_OBJECT

public:
	wgt_irp6_m_euler(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent =
			0);
	~wgt_irp6_m_euler();
	virtual void setup_ui(QGridLayout *layout, int _rows_number);

private:
	Ui::wgt_absolute_template ui;
	mrrocpp::ui::irp6_m::UiRobot *robot;

	void init();
	void move_it();
};

#endif // WGT_SPKM_INC_H
