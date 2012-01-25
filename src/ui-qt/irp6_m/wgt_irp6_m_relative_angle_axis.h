#ifndef WGT_IRP6_M_RELATIVE_ANGLE_AXIS_H
#define WGT_IRP6_M_RELATIVE_ANGLE_AXIS_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include <QVector>
#include "ui_wgt_relative_template.h"
#include "../base/WgtRelativeBase.h"

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

class wgt_irp6_m_relative_angle_axis : public WgtRelativeBase
{
	Q_OBJECT

public:
	wgt_irp6_m_relative_angle_axis(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent =
			0);
	~wgt_irp6_m_relative_angle_axis();
private:
	Ui::wgt_relative_template ui;
	mrrocpp::ui::irp6_m::UiRobot *robot;

	void init();

	void move_it();
};

#endif
