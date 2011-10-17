#include "../irp6ot_m/ui_r_irp6ot_m.h"
#include "../irp6p_m/ui_r_irp6p_m.h"
#include "../conveyor/ui_r_conveyor.h"

#include "wgt_robot_process_control.h"
#include "interface.h"
#include "ui_robot.h"
#include "allrobots.h"
#include "mp.h"

wgt_robot_process_control::wgt_robot_process_control(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent) :
		wgt_base(QString::fromStdString(robo->getName()), _interface, parent), ui(new Ui::wgt_robot_process_controlClass), robot(robo)
{
	ui->setupUi(this);
	ui->robot_label->setText(QString::fromStdString((robo->getName())));
	connect(this, SIGNAL(process_control_window_init_signal()), this, SLOT(process_control_window_init_slot()), Qt::QueuedConnection);
}

wgt_robot_process_control::~wgt_robot_process_control()
{
	delete ui;
}

Ui::wgt_robot_process_controlClass * wgt_robot_process_control::get_ui()
{
	return ui;
}

void wgt_robot_process_control::process_control_window_init()
{
	emit process_control_window_init_signal();
}

void wgt_robot_process_control::process_control_window_init_slot()
{
	init();
}

void wgt_robot_process_control::my_open(bool set_on_top)
{
	wgt_base::my_open(set_on_top);
	process_control_window_init();
}

//ECP
void wgt_robot_process_control::on_ecp_trigger_pushButton_clicked()
{
	interface.all_robots->pulse_trigger_ecp(robot);
}

// Reader
void wgt_robot_process_control::on_reader_start_pushButton_clicked()
{
	interface.all_robots->pulse_start_reader(robot);
}

void wgt_robot_process_control::on_reader_stop_pushButton_clicked()
{
	interface.all_robots->pulse_stop_reader(robot);
}

void wgt_robot_process_control::on_reader_trigger_pushButton_clicked()
{
	interface.all_robots->pulse_trigger_reader(robot);
}

// aktualizacja ustawien przyciskow
int wgt_robot_process_control::init()

{

	bool wlacz_PtButton_wnd_processes_control_all_reader_start = false;
	bool wlacz_PtButton_wnd_processes_control_all_reader_stop = false;
	bool wlacz_PtButton_wnd_processes_control_all_reader_trigger = false;

	// Dla READER'A

	ui->reader_start_pushButton->setDisabled(true);
	ui->reader_stop_pushButton->setDisabled(true);
	ui->reader_trigger_pushButton->setDisabled(true);

	// Dla irp6_on_track

	if (!(robot->is_edp_loaded())) { // edp wylaczone

	} else if (robot->state.edp.state == mrrocpp::ui::common::UI_EDP_WAITING_TO_START_READER) { // edp wlaczone reader czeka na start
		wlacz_PtButton_wnd_processes_control_all_reader_start = true;

	} else if (robot->state.edp.state == mrrocpp::ui::common::UI_EDP_WAITING_TO_STOP_READER) { // edp wlaczone reader czeka na stop
		wlacz_PtButton_wnd_processes_control_all_reader_stop = true;
		wlacz_PtButton_wnd_processes_control_all_reader_trigger = true;

	}

	if (wlacz_PtButton_wnd_processes_control_all_reader_start) {
		ui->reader_start_pushButton->setDisabled(false);

	}

	if (wlacz_PtButton_wnd_processes_control_all_reader_stop) {
		ui->reader_stop_pushButton->setDisabled(false);

	}

	if (wlacz_PtButton_wnd_processes_control_all_reader_trigger) {
		ui->reader_trigger_pushButton->setDisabled(false);

	}

	// Dla mp i ecp
	if (interface.mp->mp_state.state != interface.mp->mp_state.last_process_control_state) {
		switch (interface.mp->mp_state.state)
		{
			case ui::common::UI_MP_NOT_PERMITED_TO_RUN:
			case ui::common::UI_MP_PERMITED_TO_RUN:
				block_all_ecp_trigger_widgets();
				break;
			case ui::common::UI_MP_WAITING_FOR_START_PULSE:
				block_all_ecp_trigger_widgets();
				break;
			case ui::common::UI_MP_TASK_RUNNING:
				unblock_all_ecp_trigger_widgets();
				break;
			case ui::common::UI_MP_TASK_PAUSED:

				block_all_ecp_trigger_widgets();
				break;
			default:

				break;
		}

		interface.mp->mp_state.last_process_control_state = interface.mp->mp_state.state;

	}

	return 1;

}

int wgt_robot_process_control::block_all_ecp_trigger_widgets()

{

	ui->ecp_trigger_pushButton->setDisabled(true);

	return 1;
}

int wgt_robot_process_control::unblock_all_ecp_trigger_widgets()

{

	ui->ecp_trigger_pushButton->setDisabled(false);
	return 1;
}
