#include <boost/foreach.hpp>

#include "../irp6ot_m/ui_r_irp6ot_m.h"
#include "../irp6p_m/ui_r_irp6p_m.h"
#include "../conveyor/ui_r_conveyor.h"

#include "wgt_process_control.h"
#include "interface.h"
#include "allrobots.h"
#include "mp.h"

wgt_process_control::wgt_process_control(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
		wgt_base("Process control", _interface, parent), ui(new Ui::wgt_process_controlClass)
{

	ui->setupUi(this);

	connect(this, SIGNAL(process_control_window_init_signal()), this, SLOT(process_control_window_init_slot()), Qt::QueuedConnection);

}

wgt_process_control::~wgt_process_control()
{
	delete ui;
}

Ui::wgt_process_controlClass * wgt_process_control::get_ui()
{
	return ui;
}

void wgt_process_control::process_control_window_init()
{
	emit process_control_window_init_signal();
}

void wgt_process_control::process_control_window_init_slot()
{
	init();
}

void wgt_process_control::my_open(bool set_on_top)
{
	wgt_base::my_open(set_on_top);
	process_control_window_init();
}

void wgt_process_control::on_mp_start_pushButton_clicked()
{
	interface.mp->pulse_start_mp();
}

void wgt_process_control::on_mp_stop_pushButton_clicked()
{
	interface.mp->pulse_stop_mp();
}

void wgt_process_control::on_mp_pause_pushButton_clicked()
{
	interface.mp->pulse_pause_mp();
}

void wgt_process_control::on_mp_resume_pushButton_clicked()
{
	interface.mp->pulse_resume_mp();
}

void wgt_process_control::on_mp_trigger_pushButton_clicked()
{
	interface.mp->pulse_trigger_mp();
}

//ECP
void wgt_process_control::on_all_ecp_trigger_pushButton_clicked()
{
	interface.all_robots->pulse_trigger_ecp();
}

// Reader
void wgt_process_control::on_all_reader_start_pushButton_clicked()
{
	interface.all_robots->pulse_start_all_reader();
}

void wgt_process_control::on_all_reader_stop_pushButton_clicked()
{
	interface.all_robots->pulse_stop_all_reader();
}

void wgt_process_control::on_all_reader_trigger_pushButton_clicked()
{
	interface.all_robots->pulse_trigger_all_reader();
}

// aktualizacja ustawien przyciskow
int wgt_process_control::init()

{

	bool wlacz_PtButton_wnd_processes_control_all_reader_start = false;
	bool wlacz_PtButton_wnd_processes_control_all_reader_stop = false;
	bool wlacz_PtButton_wnd_processes_control_all_reader_trigger = false;

	// Dla READER'A

	ui->all_reader_start_pushButton->setDisabled(true);
	ui->all_reader_stop_pushButton->setDisabled(true);
	ui->all_reader_trigger_pushButton->setDisabled(true);

	BOOST_FOREACH(const mrrocpp::ui::common::robot_pair_t & robot_node, interface.robot_m)
			{

				if (!(robot_node.second->is_edp_loaded())) { // edp wylaczone

				} else if (robot_node.second->state.edp.state == mrrocpp::ui::common::UI_EDP_WAITING_TO_START_READER) { // edp wlaczone reader czeka na start
					wlacz_PtButton_wnd_processes_control_all_reader_start = true;

				} else if (robot_node.second->state.edp.state == mrrocpp::ui::common::UI_EDP_WAITING_TO_STOP_READER) { // edp wlaczone reader czeka na stop
					wlacz_PtButton_wnd_processes_control_all_reader_stop = true;
					wlacz_PtButton_wnd_processes_control_all_reader_trigger = true;

				}

			}

	// All reader's pulse buttons
	if (wlacz_PtButton_wnd_processes_control_all_reader_start) {
		ui->all_reader_start_pushButton->setDisabled(false);

	}

	if (wlacz_PtButton_wnd_processes_control_all_reader_stop) {
		ui->all_reader_stop_pushButton->setDisabled(false);

	}

	if (wlacz_PtButton_wnd_processes_control_all_reader_trigger) {
		ui->all_reader_trigger_pushButton->setDisabled(false);

	}

	// Dla mp i ecp
	if (interface.mp->mp_state.state != interface.mp->mp_state.last_process_control_state) {
		switch (interface.mp->mp_state.state)
		{
			case ui::common::UI_MP_NOT_PERMITED_TO_RUN:
			case ui::common::UI_MP_PERMITED_TO_RUN:
				ui->mp_start_pushButton->setDisabled(true);
				ui->mp_stop_pushButton->setDisabled(true);
				ui->mp_pause_pushButton->setDisabled(true);
				ui->mp_resume_pushButton->setDisabled(true);
				ui->mp_trigger_pushButton->setDisabled(true);

				block_all_ecp_trigger_widgets();
				break;
			case ui::common::UI_MP_WAITING_FOR_START_PULSE:

				ui->mp_start_pushButton->setDisabled(false);
				ui->mp_stop_pushButton->setDisabled(true);
				ui->mp_pause_pushButton->setDisabled(true);
				ui->mp_resume_pushButton->setDisabled(true);
				ui->mp_trigger_pushButton->setDisabled(true);

				block_all_ecp_trigger_widgets();
				break;
			case ui::common::UI_MP_TASK_RUNNING:

				ui->mp_start_pushButton->setDisabled(true);
				ui->mp_stop_pushButton->setDisabled(false);
				ui->mp_pause_pushButton->setDisabled(false);
				ui->mp_resume_pushButton->setDisabled(true);
				ui->mp_trigger_pushButton->setDisabled(false);

				unblock_all_ecp_trigger_widgets();
				break;
			case ui::common::UI_MP_TASK_PAUSED:

				ui->mp_start_pushButton->setDisabled(true);
				ui->mp_stop_pushButton->setDisabled(false);
				ui->mp_pause_pushButton->setDisabled(true);
				ui->mp_resume_pushButton->setDisabled(false);
				ui->mp_trigger_pushButton->setDisabled(true);

				block_all_ecp_trigger_widgets();
				break;
			default:

				break;
		}

		interface.mp->mp_state.last_process_control_state = interface.mp->mp_state.state;

	}

	return 1;

}

int wgt_process_control::block_all_ecp_trigger_widgets()

{

	ui->all_ecp_trigger_pushButton->setDisabled(true);

	return 1;
}

int wgt_process_control::unblock_all_ecp_trigger_widgets()

{

	ui->all_ecp_trigger_pushButton->setDisabled(false);
	return 1;
}
