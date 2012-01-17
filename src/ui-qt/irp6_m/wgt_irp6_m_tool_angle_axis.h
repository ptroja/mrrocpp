#ifndef WGT_IRP6_M_TOOL_ANGLE_AXIS_H
#define WGT_IRP6_M_TOOL_ANGLE_AXIS_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include <QVector>

#include "../base/WgtToolBase.h"

#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "../../base/lib/mrmath/mrmath.h"

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

namespace mrrocpp {
namespace lib {
class Xyz_Angle_Axis_vector;
}
}

class wgt_irp6_m_tool_angle_axis : public WgtToolBase
{
	Q_OBJECT

public:
	wgt_irp6_m_tool_angle_axis(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent =
			0);
	~wgt_irp6_m_tool_angle_axis();

private:
	mrrocpp::ui::irp6_m::UiRobot *robot;

	mrrocpp::lib::Xyz_Angle_Axis_vector tool_vector;
	void init();

	void get_desired_position();
	void move_it();

};

#endif // WGT_SPKM_INC_H
