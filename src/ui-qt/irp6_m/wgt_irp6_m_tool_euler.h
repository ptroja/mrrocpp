#ifndef WGT_IRP6_M_TOOL_EULER_H
#define WGT_IRP6_M_TOOL_EULER_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include <QVector>
#include "ui_wgt_tool_template.h"
#include "../base/WgtToolBase.h"

#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

#include "../../base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ui {
namespace common {
//class Interface;
class UiRobot;
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

class wgt_irp6_m_tool_euler : public WgtToolBase
{
	Q_OBJECT

public:
	wgt_irp6_m_tool_euler(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent =
			0);
	~wgt_irp6_m_tool_euler();
private:
	mrrocpp::ui::irp6_m::UiRobot *robot;

	void init();

	void get_desired_position();
	void move_it();
	mrrocpp::lib::Xyz_Euler_Zyz_vector tool_vector;

};

#endif // WGT_SPKM_INC_H
