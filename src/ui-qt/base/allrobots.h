

#ifndef ALLROBOTS_H_
#define ALLROBOTS_H_

#include <boost/shared_ptr.hpp>
#include "ui.h"

class MainWindow;

namespace mrrocpp {
namespace ui {
namespace common {
class UiRobot;
class Interface;
class Mp;


class AllRobots
{
public:
	AllRobots(Interface *iface);

	int EDP_all_robots_create();
	int EDP_all_robots_slay();
	int EDP_all_robots_synchronise();

	//Reader pulse
	int pulse_start_all_reader();
	int pulse_stop_all_reader();
	int pulse_trigger_all_reader();

	//Reader pulse
	int pulse_start_reader(UiRobot *robot);
	int pulse_stop_reader(UiRobot *robot);
	int pulse_trigger_reader(UiRobot *robot);

	void set_edp_state();

	void manage_interface();
	//ECP pulse
	int pulse_trigger_ecp();

	//ECP pulse
	int pulse_trigger_ecp(UiRobot *robot);

	int all_robots_move_to_synchro_position();
	int all_robots_move_to_front_position();
	int all_robots_move_to_preset_position_0();
	int all_robots_move_to_preset_position_1();
	int all_robots_move_to_preset_position_2();

	bool is_any_robot_active();
	bool are_all_active_robots_loaded();
	bool is_any_active_robot_loaded();
	bool are_all_loaded_robots_synchronised();
	bool is_any_loaded_robot_synchronised();

	UI_ALL_EDPS_STATE all_edps;
	UI_ALL_EDPS_STATE all_edps_last_manage_interface_state;
	UI_ALL_EDPS_SYNCHRO_STATE all_edps_synchro;
	UI_ALL_EDPS_SYNCHRO_STATE all_edps_synchro_last_manage_interface_state;

private:
	boost::shared_ptr<MainWindow> mw;
	Interface *interface;

};

}
}
}

#endif /* ALLROBOTS_H_ */
