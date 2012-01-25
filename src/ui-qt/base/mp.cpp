#include "mp.h"
#include "allrobots.h"
#include "mainwindow.h"
#include "interface.h"
#include "ui_robot.h"
#include "ui_mainwindow.h"
#include <boost/foreach.hpp>

namespace mrrocpp {
namespace ui {
namespace common {

Mp::Mp(Interface & iface) :
		interface(iface)
{
	mp_state.state = UI_MP_NOT_PERMITED_TO_RUN; // mp wylaczone
	mp_state.last_process_control_state = UI_MP_STATE_NOT_KNOWN;
	mp_state.last_manage_interface_state = UI_MP_STATE_NOT_KNOWN;

	mp_state.pid = -1;
}

void Mp::MPup()
{

	interface.main_eb->command(boost::bind(&ui::common::Mp::MPup_int, this));

}

int Mp::MPup_int()
{
	interface.set_ui_state_notification(UI_N_PROCESS_CREATION);

	try {

		if (mp_state.pid == -1) {

			std::string mp_network_pulse_attach_point("/dev/name/global/");
			mp_network_pulse_attach_point += mp_state.network_pulse_attach_point;

			// sprawdzenie czy nie jest juz zarejestrowany serwer komunikacyjny MP
			if (access(mp_network_pulse_attach_point.c_str(), R_OK) == 0) {
				interface.ui_msg->message(lib::NON_FATAL_ERROR, "mp already exists");
			} else if (interface.check_node_existence(mp_state.node_name, "mp")) {
				mp_state.pid = interface.config->process_spawn(lib::MP_SECTION);

				if (mp_state.pid > 0) {

					mp_state.MP = (boost::shared_ptr <lib::agent::RemoteAgent>) new lib::agent::RemoteAgent(lib::MP_SECTION);
					mp_state.pulse =
							(boost::shared_ptr <lib::agent::OutputBuffer <char> >) new lib::agent::OutputBuffer <char>(*mp_state.MP, "MP_PULSE");

					interface.teachingstate = ui::common::MP_RUNNING;

					mp_state.state = ui::common::UI_MP_WAITING_FOR_START_PULSE; // mp wlaczone

					interface.raise_process_control_window();
				} else {
					BOOST_THROW_EXCEPTION(lib::exception::system_error());
				}
				interface.manage_interface();

			}
		}

	} catch (...) {
		interface.ui_state = 2;
	}
	return 1;

}

void Mp::set_mp_state()
{
	if ((interface.all_robots->all_edps == UI_ALL_EDPS_NONE_ACTIVATED)
			|| ((interface.all_robots->all_edps == UI_ALL_EDPS_ALL_LOADED)
					&& (interface.all_robots->all_edps_synchro == UI_ALL_EDPS_ALL_SYNCHRONISED))) {
		if ((mp_state.state == UI_MP_NOT_PERMITED_TO_RUN) && (interface.is_mp_and_ecps_active))
			mp_state.state = UI_MP_PERMITED_TO_RUN; // pozwol na uruchomienie mp

	} else if (mp_state.state == UI_MP_PERMITED_TO_RUN)
		mp_state.state = UI_MP_NOT_PERMITED_TO_RUN; // nie pozwol na uruchomienie mp

}

void Mp::MPslay()
{
	try {
		if (mp_state.pid != -1) {

			if ((mp_state.state == ui::common::UI_MP_TASK_RUNNING)
					|| (mp_state.state == ui::common::UI_MP_TASK_PAUSED)) {

				pulse_stop_mp();
			}

			if (mp_state.pulse) {
				mp_state.pulse.reset();
			} else {
				std::cerr << "MP pulse not connected?" << std::endl;
			}

			if (mp_state.MP) {
				mp_state.MP.reset();
			} else {
				std::cerr << "MP not connected?" << std::endl;
			}

			// 	printf("dddd: %d\n", SignalKill(ini_con->mp-
			// 	printf("mp slay\n");

			//	SignalKill(mp.node_nr, mp.pid, 0, SIGTERM, 0, 0);

			interface.block_sigchld();
			if (kill(mp_state.pid, SIGTERM) == -1) {
				perror("kill()");
			} else {
				//    		int status;
				//    		if (waitpid(EDP_MASTER_Pid, &status, 0) == -1) {
				//    			perror("waitpid()");
				//    		}
				interface.wait_for_child_termination(mp_state.pid, true);
			}
			interface.unblock_sigchld();
			mp_state.state = ui::common::UI_MP_PERMITED_TO_RUN; // mp wylaczone

		}
		// delay(1000);
		// 	kill(mp_pid,SIGTERM);
		// 	printf("mp pupa po kill\n");
		mp_state.pid = -1;

		BOOST_FOREACH(const robot_pair_t & robot_node, interface.robot_m)
				{
					robot_node.second->deactivate_ecp_trigger();
				}

		// modyfikacja menu
		interface.manage_interface();
		interface.wgt_pc->process_control_window_init();

		BOOST_FOREACH(const common::robot_pair_t & robot_node, interface.robot_m)
				{
					if ((robot_node.second->state.is_active) && (robot_node.second->is_edp_loaded())) {
						robot_node.second->get_wgt_robot_pc()->process_control_window_init();
					}
				}
		//wgt_pc->dwgt->raise();
	} catch (...) {
		interface.ui_state = 2;
	}
}

void Mp::pulse_start_mp()

{
	try {
		if (mp_state.state == ui::common::UI_MP_WAITING_FOR_START_PULSE) {

			mp_state.state = ui::common::UI_MP_TASK_RUNNING; // czekanie na stop

			// close_all_windows
			BOOST_FOREACH(const ui::common::robot_pair_t & robot_node, interface.robot_m)
					{
						robot_node.second->close_all_windows();
					}

			execute_mp_pulse(MP_START);

			interface.wgt_pc->process_control_window_init();
			BOOST_FOREACH(const common::robot_pair_t & robot_node, interface.robot_m)
					{
						if ((robot_node.second->state.is_active) && (robot_node.second->is_edp_loaded())) {
							robot_node.second->get_wgt_robot_pc()->process_control_window_init();
						}
					}
			//wgt_pc->dwgt->raise();
			manage_interface();
		}
	} catch (...) {
		interface.ui_state = 2;
	}

}

void Mp::pulse_stop_mp()

{
	try {
		if ((mp_state.state == ui::common::UI_MP_TASK_RUNNING) || (mp_state.state == ui::common::UI_MP_TASK_PAUSED)) {

			mp_state.state = ui::common::UI_MP_WAITING_FOR_START_PULSE; // czekanie na stop

			execute_mp_pulse(MP_STOP);

			interface.manage_interface();
		}
	} catch (...) {
		interface.ui_state = 2;
	}
}

void Mp::pulse_pause_mp()

{
	try {
		if (mp_state.state == ui::common::UI_MP_TASK_RUNNING) {

			mp_state.state = ui::common::UI_MP_TASK_PAUSED; // czekanie na stop

			execute_mp_pulse(MP_PAUSE);

			interface.manage_interface();
		}
	} catch (...) {
		interface.ui_state = 2;
	}
}

void Mp::pulse_resume_mp()

{
	try {
		if (mp_state.state == ui::common::UI_MP_TASK_PAUSED) {

			mp_state.state = ui::common::UI_MP_TASK_RUNNING; // czekanie na stop

			execute_mp_pulse(MP_RESUME);

			interface.manage_interface();
		}
	} catch (...) {
		interface.ui_state = 2;
	}
}

void Mp::pulse_trigger_mp()

{
	try {
		if (mp_state.state == ui::common::UI_MP_TASK_RUNNING) {

			execute_mp_pulse(MP_TRIGGER);

			interface.manage_interface();
		}
	} catch (...) {
		interface.ui_state = 2;
	}
}

void Mp::execute_mp_pulse(char pulse_code)
{
	try {
		// printf("w send pulse\n");
		if (mp_state.pulse) {
			mp_state.pulse->Send(pulse_code);
		}
	} catch (...) {
		interface.ui_state = 2;
	}
}

void Mp::manage_interface()
{
	MainWindow & mw = *interface.mw;

	if (mp_state.state != mp_state.last_manage_interface_state) {
		// wlasciwosci menu task_menu
		switch (mp_state.state)
		{
			case common::UI_MP_NOT_PERMITED_TO_RUN:
				//		std::cout << "UI_MP_NOT_PERMITED_TO_RUN" << std::endl;
				mw.get_ui()->label_mp_notification->setText("NOT_PERMITED_TO_RUN");
				mw.getMenuBar()->actionMP_Load->setEnabled(false);
				mw.getMenuBar()->actionMP_Unload->setEnabled(false);

				break;
			case common::UI_MP_PERMITED_TO_RUN:
				//	std::cout << "UI_MP_PERMITED_TO_RUN" << std::endl;
				mw.get_ui()->label_mp_notification->setText("PERMITED_TO_RUN");

				mw.getMenuBar()->actionMP_Load->setEnabled(true);
				mw.getMenuBar()->actionMP_Unload->setEnabled(false);

				break;
			case common::UI_MP_WAITING_FOR_START_PULSE:
				//	std::cout << "UI_MP_WAITING_FOR_START_PULSE" << std::endl;
				mw.get_ui()->label_mp_notification->setText("WAITING_FOR_START_PULSE");

				mw.getMenuBar()->actionMP_Load->setEnabled(false);
				mw.getMenuBar()->actionMP_Unload->setEnabled(true);

				break;
			case common::UI_MP_TASK_RUNNING:
				//	std::cout << "UI_MP_TASK_RUNNING" << std::endl;

				mw.get_ui()->label_mp_notification->setText("TASK_RUNNING");

				mw.getMenuBar()->actionMP_Load->setEnabled(false);
				mw.getMenuBar()->actionMP_Unload->setEnabled(false);

				break;
			case common::UI_MP_TASK_PAUSED:
				//  std::cout << "UI_MP_TASK_PAUSED" << std::endl;
				mw.get_ui()->label_mp_notification->setText("TASK_PAUSED");

				mw.getMenuBar()->actionMP_Load->setEnabled(false);
				mw.getMenuBar()->actionMP_Unload->setEnabled(false);

				break;
			default:
				break;
		}
		mp_state.last_manage_interface_state = mp_state.state;
	}
}

}
}
}
