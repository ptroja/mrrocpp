#include "allrobots.h"
#include "mainwindow.h"
#include "interface.h"
#include "ui_robot.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <boost/foreach.hpp>
#include "mp.h"

namespace mrrocpp {
namespace ui {
namespace common {

AllRobots::AllRobots(Interface *iface) :
		all_edps(UI_ALL_EDPS_NONE_LOADED), all_edps_last_manage_interface_state(UI_ALL_EDPS_STATE_NOT_KNOWN), all_edps_synchro(UI_ALL_EDPS_NONE_SYNCHRONISED), all_edps_synchro_last_manage_interface_state(UI_ALL_EDPS_SYNCHRO_STATE_NOT_KNOWN), interface(iface)
{
	mw = interface->mw;
}

int AllRobots::EDP_all_robots_create()
{
	BOOST_FOREACH(const robot_pair_t & robot_node, interface->robot_m)
			{
				if (!robot_node.second->get_wgt_robot_pc() && (robot_node.second->state.is_active))
					robot_node.second->open_robot_process_control_window();

				robot_node.second->edp_create();
			}

	return 1;

}

void AllRobots::manage_interface()
{
	if (all_edps_synchro != all_edps_synchro_last_manage_interface_state) {
		switch (all_edps_synchro)
		{
			case UI_ALL_EDPS_SYNCHRO_STATE_NOT_KNOWN:
				mw->get_ui()->label_all_edps_synchro_notification->setText("NOT_KNOWN");
				break;
			case UI_ALL_EDPS_SYNCHRO_NONE_EDP_LOADED:
				mw->get_ui()->label_all_edps_synchro_notification->setText("NONE EDP LOADED");

				mw->getMenuBar()->actionall_Synchronisation->setEnabled(false);

				break;
			case UI_ALL_EDPS_NONE_SYNCHRONISED:
				mw->get_ui()->label_all_edps_synchro_notification->setText("NONE_SYNCHRONISED");
				break;
			case UI_ALL_EDPS_SOME_SYNCHRONISED:
				mw->get_ui()->label_all_edps_synchro_notification->setText("SOME_SYNCHRONISED");
				break;
			case UI_ALL_EDPS_ALL_SYNCHRONISED:
				mw->get_ui()->label_all_edps_synchro_notification->setText("ALL_SYNCHRONISED");
				mw->getMenuBar()->actionall_Synchronisation->setEnabled(false);

				break;
		}
	}

	if ((all_edps != all_edps_last_manage_interface_state)
			|| (all_edps_synchro != all_edps_synchro_last_manage_interface_state)
			|| (interface->mp->mp_state.state != interface->mp->mp_state.last_manage_interface_state)) {

		if (((all_edps == UI_ALL_EDPS_NONE_ACTIVATED)
				&& ((interface->mp->mp_state.state == UI_MP_NOT_PERMITED_TO_RUN)
						|| (interface->mp->mp_state.state == UI_MP_PERMITED_TO_RUN)))
				|| (all_edps == UI_ALL_EDPS_NONE_LOADED)) {

			mw->getMenuBar()->actionConfiguration->setEnabled(true);

		} else {
			mw->getMenuBar()->actionConfiguration->setEnabled(false);

		}

		switch (all_edps)
		{
			case UI_ALL_EDPS_NONE_ACTIVATED:
				mw->get_ui()->label_all_edps_notification->setText("NONE_ACTIVATED");
				mw->getMenuBar()->menuall_Preset_Positions->setEnabled(false);
				mw->getMenuBar()->menuRobot->setEnabled(false);
				mw->getMenuBar()->menuAll_Robots->setEnabled(false);
				mw->getMenuBar()->actionall_EDP_Unload->setEnabled(false);
				mw->getMenuBar()->actionall_EDP_Load->setEnabled(false);

				break;
			case UI_ALL_EDPS_NONE_LOADED:
				mw->get_ui()->label_all_edps_notification->setText("NONE_LOADED");
				mw->getMenuBar()->menuRobot->setEnabled(true);
				mw->getMenuBar()->menuAll_Robots->setEnabled(true);
				mw->getMenuBar()->actionall_EDP_Load->setEnabled(true);
				mw->getMenuBar()->menuall_Preset_Positions->setEnabled(false);
				mw->getMenuBar()->actionall_EDP_Unload->setEnabled(false);

				break;
			case UI_ALL_EDPS_SOME_LOADED:
				mw->get_ui()->label_all_edps_notification->setText("SOME_LOADED");
				mw->getMenuBar()->actionall_EDP_Unload->setEnabled(true);
				mw->getMenuBar()->actionall_EDP_Load->setEnabled(true);
				mw->getMenuBar()->menuRobot->setEnabled(true);
				mw->getMenuBar()->menuAll_Robots->setEnabled(true);

				switch (all_edps_synchro)
				{
					case UI_ALL_EDPS_SYNCHRO_STATE_NOT_KNOWN:
					case UI_ALL_EDPS_NONE_SYNCHRONISED:
						mw->getMenuBar()->menuall_Preset_Positions->setEnabled(false);
						mw->getMenuBar()->actionall_Synchronisation->setEnabled(true);

						break;
					case UI_ALL_EDPS_SOME_SYNCHRONISED:
						mw->getMenuBar()->menuall_Preset_Positions->setEnabled(true);
						mw->getMenuBar()->actionall_Synchronisation->setEnabled(true);

						break;
					case UI_ALL_EDPS_ALL_SYNCHRONISED:
						mw->getMenuBar()->menuall_Preset_Positions->setEnabled(true);

						break;
					default:
						break;
				}

				break;

			case UI_ALL_EDPS_ALL_LOADED:
				mw->get_ui()->label_all_edps_notification->setText("ALL_LOADED		");
				mw->getMenuBar()->menuRobot->setEnabled(true);
				mw->getMenuBar()->menuAll_Robots->setEnabled(true);
				mw->getMenuBar()->actionall_EDP_Load->setEnabled(false);

				switch (all_edps_synchro)
				{
					case UI_ALL_EDPS_SYNCHRO_STATE_NOT_KNOWN:
					case UI_ALL_EDPS_NONE_SYNCHRONISED:
						mw->getMenuBar()->actionall_EDP_Unload->setEnabled(true);
						mw->getMenuBar()->menuall_Preset_Positions->setEnabled(false);
						mw->getMenuBar()->actionall_Synchronisation->setEnabled(true);

						break;
					case UI_ALL_EDPS_SOME_SYNCHRONISED:
						mw->getMenuBar()->actionall_EDP_Unload->setEnabled(true);
						mw->getMenuBar()->menuall_Preset_Positions->setEnabled(true);
						mw->getMenuBar()->actionall_Synchronisation->setEnabled(true);

						break;
					case UI_ALL_EDPS_ALL_SYNCHRONISED:
						mw->getMenuBar()->menuall_Preset_Positions->setEnabled(true);

						switch (interface->mp->mp_state.state)
						{
							case common::UI_MP_NOT_PERMITED_TO_RUN:
								mw->getMenuBar()->actionall_EDP_Unload->setEnabled(true);
								mw->getMenuBar()->menuall_Preset_Positions->setEnabled(true);

								break;
							case common::UI_MP_PERMITED_TO_RUN:
								mw->getMenuBar()->actionall_EDP_Unload->setEnabled(true);
								mw->getMenuBar()->menuall_Preset_Positions->setEnabled(true);

								break;
							case common::UI_MP_WAITING_FOR_START_PULSE:
								mw->getMenuBar()->actionall_EDP_Unload->setEnabled(false);
								mw->getMenuBar()->menuall_Preset_Positions->setEnabled(true);

								break;
							case common::UI_MP_TASK_RUNNING:
							case common::UI_MP_TASK_PAUSED:
								mw->getMenuBar()->actionall_EDP_Unload->setEnabled(false);
								mw->getMenuBar()->menuall_Preset_Positions->setEnabled(false);

								break;
							default:
								break;
						}
						break;
					default:
						break;
				}
				break;
			default:
				break;
		}
		all_edps_last_manage_interface_state = all_edps;
		all_edps_synchro_last_manage_interface_state = all_edps_synchro;
	}
}

void AllRobots::set_edp_state()
{
	// wyznaczenie stanu wszytkich EDP abstahujac od MP

	// jesli wszytkie sa nieaktywne
	if (!is_any_robot_active())
		all_edps = UI_ALL_EDPS_NONE_ACTIVATED;
	else if (are_all_active_robots_loaded())
		all_edps = UI_ALL_EDPS_ALL_LOADED;
	else if (is_any_active_robot_loaded())
		all_edps = UI_ALL_EDPS_SOME_LOADED;
	else
		all_edps = UI_ALL_EDPS_NONE_LOADED;

	if ((all_edps == UI_ALL_EDPS_NONE_ACTIVATED) || (all_edps == UI_ALL_EDPS_NONE_LOADED))
		all_edps_synchro = UI_ALL_EDPS_SYNCHRO_NONE_EDP_LOADED;
	else {
		// jesli wszytkie sa zsynchronizowane
		if (are_all_loaded_robots_synchronised())
			all_edps_synchro = UI_ALL_EDPS_ALL_SYNCHRONISED;
		else if (is_any_loaded_robot_synchronised())
			all_edps_synchro = UI_ALL_EDPS_SOME_SYNCHRONISED;
		else
			all_edps_synchro = UI_ALL_EDPS_NONE_SYNCHRONISED;
	}
}

int AllRobots::EDP_all_robots_slay()
{
	BOOST_FOREACH(const robot_pair_t & robot_node, interface->robot_m)
			{
				robot_node.second->delete_robot_process_control_window();
				robot_node.second->EDP_slay_int();
			}

	return 1;

}

int AllRobots::EDP_all_robots_synchronise()

{

	BOOST_FOREACH(const robot_pair_t & robot_node, interface->robot_m)
			{
				robot_node.second->synchronise();
			}

	return 1;

}

//Reader pulse
int AllRobots::pulse_start_all_reader()
{
	BOOST_FOREACH(const robot_pair_t & robot_node, interface->robot_m)
			{
				robot_node.second->pulse_reader_start_exec_pulse();
			}

	interface->manage_pc();

	return 1;
}

int AllRobots::pulse_stop_all_reader()
{
	BOOST_FOREACH(const robot_pair_t & robot_node, interface->robot_m)
			{
				robot_node.second->pulse_reader_stop_exec_pulse();
			}
	interface->manage_pc();
	return 1;
}

int AllRobots::pulse_trigger_all_reader()
{
	BOOST_FOREACH(const robot_pair_t & robot_node, interface->robot_m)
			{
				robot_node.second->pulse_reader_trigger_exec_pulse();
			}

	return 1;
}

int AllRobots::pulse_start_reader(UiRobot *robot)
{
	robot->pulse_reader_start_exec_pulse();
	interface->manage_pc();

	return 1;
}

int AllRobots::pulse_stop_reader(UiRobot *robot)
{
	robot->pulse_reader_stop_exec_pulse();
	interface->manage_pc();
	return 1;
}

int AllRobots::pulse_trigger_reader(UiRobot *robot)
{
	robot->pulse_reader_trigger_exec_pulse();
	return 1;
}

//ECP pulse
int AllRobots::pulse_trigger_ecp()
{

	BOOST_FOREACH(const robot_pair_t & robot_node, interface->robot_m)
			{
				robot_node.second->pulse_ecp();
			}

	return 1;
}

//ECP pulse
int AllRobots::pulse_trigger_ecp(UiRobot *robot)
{

	robot->pulse_ecp();
	return 1;
}

int AllRobots::all_robots_move_to_synchro_position()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((interface->mp->mp_state.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN)
			|| (interface->mp->mp_state.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (interface->mp->mp_state.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, interface->robot_m)
				{
					if (robot_node.second->check_synchronised_and_loaded()) {
						robot_node.second->move_to_synchro_position();
					}
				}

	}

	return 1;

}

int AllRobots::all_robots_move_to_preset_position_1()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((interface->mp->mp_state.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN)
			|| (interface->mp->mp_state.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (interface->mp->mp_state.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, interface->robot_m)
				{
					if (robot_node.second->check_synchronised_and_loaded()) {
						robot_node.second->move_to_preset_position(1);
					}
				}

	}

	return 1;

}

int AllRobots::all_robots_move_to_preset_position_2()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((interface->mp->mp_state.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN)
			|| (interface->mp->mp_state.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (interface->mp->mp_state.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, interface->robot_m)
				{
					if (robot_node.second->check_synchronised_and_loaded()) {
						robot_node.second->move_to_preset_position(2);
					}
				}

	}
	return 1;

}

int AllRobots::all_robots_move_to_preset_position_0()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((interface->mp->mp_state.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN)
			|| (interface->mp->mp_state.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (interface->mp->mp_state.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, interface->robot_m)
				{
					if (robot_node.second->check_synchronised_and_loaded()) {
						robot_node.second->move_to_preset_position(0);
					}
				}

	}

	return 1;

}

int AllRobots::all_robots_move_to_front_position()

{

	// jesli MP nie pracuje (choc moze byc wlaczone)
	if ((interface->mp->mp_state.state == ui::common::UI_MP_NOT_PERMITED_TO_RUN)
			|| (interface->mp->mp_state.state == ui::common::UI_MP_PERMITED_TO_RUN)
			|| (interface->mp->mp_state.state == ui::common::UI_MP_WAITING_FOR_START_PULSE)) {

		BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, interface->robot_m)
				{
					if (robot_node.second->check_synchronised_and_loaded()) {
						robot_node.second->move_to_front_position();
					}
				}

	}
	return 1;

}

bool AllRobots::is_any_robot_active()
{
	BOOST_FOREACH(const common::robot_pair_t & robot_node, interface->robot_m)
			{
				if (robot_node.second->state.is_active) {
					return true;
				}
			}

	return false;
}

bool AllRobots::are_all_active_robots_loaded()
{

	BOOST_FOREACH(const common::robot_pair_t & robot_node, interface->robot_m)
			{
				if ((robot_node.second->state.is_active) && (!(robot_node.second->is_edp_loaded()))) {

					return false;
				}
			}

	return true;
}

bool AllRobots::is_any_active_robot_loaded()
{

	BOOST_FOREACH(const common::robot_pair_t & robot_node, interface->robot_m)
			{
				if ((robot_node.second->state.is_active) && (robot_node.second->is_edp_loaded())) {
					return true;
				}
			}

	return false;
}

bool AllRobots::are_all_loaded_robots_synchronised()
{

	BOOST_FOREACH(const common::robot_pair_t & robot_node, interface->robot_m)
			{
				if ((robot_node.second->is_edp_loaded()) && (!(robot_node.second->state.edp.is_synchronised))) {

					return false;
				}
			}

	return true;
}

bool AllRobots::is_any_loaded_robot_synchronised()
{
	BOOST_FOREACH(const common::robot_pair_t & robot_node, interface->robot_m)
			{
				if ((robot_node.second->is_edp_loaded()) && (robot_node.second->state.edp.is_synchronised)) {
					return true;
				}
			}

	return false;
}

}
}
}
