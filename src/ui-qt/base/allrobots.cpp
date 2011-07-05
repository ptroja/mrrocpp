int Interface::EDP_all_robots_create()
{
	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					if (!robot_node.second->get_wgt_robot_pc() && (robot_node.second->state.is_active))
						robot_node.second->set_robot_process_control_window(new wgt_robot_process_control(*this, robot_node.second));

					robot_node.second->edp_create();
				}

	return 1;

}

int Interface::EDP_all_robots_slay()
{
	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->delete_robot_process_control_window();
					robot_node.second->EDP_slay_int();
				}

	return 1;

}

int Interface::EDP_all_robots_synchronise()

{

	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->synchronise();
				}

	return 1;

}


//Reader pulse
int Interface::pulse_start_all_reader()
{
	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->pulse_reader_start_exec_pulse();
				}

	manage_pc();

	return 1;
}

int Interface::pulse_stop_all_reader()
{
	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->pulse_reader_stop_exec_pulse();
				}
	manage_pc();
	return 1;
}

int Interface::pulse_trigger_all_reader()
{
	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->pulse_reader_trigger_exec_pulse();
				}

	return 1;
}


int Interface::pulse_start_reader(common::UiRobot *robot)
{
	robot->pulse_reader_start_exec_pulse();
	manage_pc();

	return 1;
}

int Interface::pulse_stop_reader(common::UiRobot *robot)
{
	robot->pulse_reader_stop_exec_pulse();
	manage_pc();
	return 1;
}

int Interface::pulse_trigger_reader(common::UiRobot *robot)
{
	robot->pulse_reader_trigger_exec_pulse();
	return 1;
}



//ECP pulse
int Interface::pulse_trigger_ecp()
{

	BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
				{
					robot_node.second->pulse_ecp();
				}

	return 1;
}


//ECP pulse
int Interface::pulse_trigger_ecp(common::UiRobot *robot)
{

	robot->pulse_ecp();
	return 1;
}


int Interface::all_robots_move_to_synchro_position()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((mp.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN) || (mp.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (mp.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
					{
						if (robot_node.second->check_synchronised_and_loaded()) {
							robot_node.second->move_to_synchro_position();
						}
					}

	}

	return 1;

}

int Interface::all_robots_move_to_preset_position_1()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((mp.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN) || (mp.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (mp.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
					{
						if (robot_node.second->check_synchronised_and_loaded()) {
							robot_node.second->move_to_preset_position(1);
						}
					}

	}

	return 1;

}

int Interface::all_robots_move_to_preset_position_2()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((mp.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN) || (mp.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (mp.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
					{
						if (robot_node.second->check_synchronised_and_loaded()) {
							robot_node.second->move_to_preset_position(2);
						}
					}

	}
	return 1;

}

int Interface::all_robots_move_to_preset_position_0()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((mp.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN) || (mp.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (mp.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
					{
						if (robot_node.second->check_synchronised_and_loaded()) {
							robot_node.second->move_to_preset_position(0);
						}
					}

	}

	return 1;

}

int Interface::all_robots_move_to_front_position()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((mp.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN) || (mp.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (mp.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, robot_m)
					{
						if (robot_node.second->check_synchronised_and_loaded()) {
							robot_node.second->move_to_front_position();
						}
					}

	}
	return 1;

}


bool Interface::is_any_robot_active()
{
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if (robot_node.second->state.is_active) {
						return true;
					}
				}

	return false;
}

bool Interface::are_all_active_robots_loaded()
{

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if ((robot_node.second->state.is_active) && (robot_node.second->state.edp.state <= 0)) {

						return false;
					}
				}

	return true;
}

bool Interface::is_any_active_robot_loaded()
{

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if ((robot_node.second->state.is_active) && (robot_node.second->state.edp.state > 0)) {
						return true;
					}
				}

	return false;
}

bool Interface::are_all_loaded_robots_synchronised()
{

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if ((robot_node.second->state.edp.state > 0) && (!(robot_node.second->state.edp.is_synchronised))) {

						return false;
					}
				}

	return true;
}

bool Interface::is_any_loaded_robot_synchronised()
{
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
				{
					if ((robot_node.second->state.edp.state > 0) && (robot_node.second->state.edp.is_synchronised)) {
						return true;
					}
				}

	return false;
}
