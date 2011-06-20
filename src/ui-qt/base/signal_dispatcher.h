#ifndef SIGNALDISPATCHER_H
#define SIGNALDISPATCHER_H
#include <QObject>


class wgt_base;
class wgt_robot_process_control;

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class UiRobot;
}
}
}
namespace mrrocpp{
namespace ui{
namespace irp6ot_m{
class UiRobot;
}
}
}

namespace Ui
{

class SignalDispatcher : public QObject
{
Q_OBJECT

public:
	SignalDispatcher(mrrocpp::ui::common::Interface& iface);
	~SignalDispatcher();


public slots:
	void on_EDP_Load_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_EDP_Unload_triggered(mrrocpp::ui::common::UiRobot *robot);

	void on_robot_process_control_triggered(mrrocpp::ui::common::UiRobot *robot);

	void on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Pre_Synchro_Moves_Motors_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Move_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Move_Joints_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Absolute_Moves_Motors_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Joints_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Command_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Configuration_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Absolute_Moves_Xyz_Euler_Zyz_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Absolute_Moves_Xyz_Angle_Axis_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_External_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Clear_Fault_triggered(mrrocpp::ui::common::UiRobot *robot);

	void on_Motors_triggered(mrrocpp::ui::common::UiRobot *robot);

	void on_Relative_Xyz_Angle_Axis_triggered(mrrocpp::ui::common::UiRobot *robot);

	void on_Synchro_Position_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Front_Position_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Position_0_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Position_1_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Position_2_triggered(mrrocpp::ui::common::UiRobot *robot);

	void on_Tool_Xyz_Euler_Zyz_triggered(mrrocpp::ui::common::UiRobot *robot);
	void on_Tool_Xyz_Angle_Axis_triggered(mrrocpp::ui::common::UiRobot *robot);

	void open_new_window(wgt_base *window, bool set_on_top=true);

private:
	mrrocpp::ui::common::Interface &interface;

};

}

#endif
