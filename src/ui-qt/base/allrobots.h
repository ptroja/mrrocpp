

#ifndef ALLROBOTS_H_
#define ALLROBOTS_H_


class AllRobots
{
	int EDP_all_robots_create();
	int EDP_all_robots_slay();
	int EDP_all_robots_synchronise();

	//Reader pulse
	int pulse_start_all_reader();
	int pulse_stop_all_reader();
	int pulse_trigger_all_reader();

	//Reader pulse
	int pulse_start_reader(common::UiRobot *robot);
	int pulse_stop_reader(common::UiRobot *robot);
	int pulse_trigger_reader(common::UiRobot *robot);

	UI_ALL_EDPS_STATE all_edps;
	UI_ALL_EDPS_STATE all_edps_last_manage_interface_state;
	UI_ALL_EDPS_SYNCHRO_STATE all_edps_synchro;
	UI_ALL_EDPS_SYNCHRO_STATE all_edps_synchro_last_manage_interface_state;

	//ECP pulse
	int pulse_trigger_ecp();

	//ECP pulse
	int pulse_trigger_ecp(common::UiRobot *robot);

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

};
#endif /* ALLROBOTS_H_ */
