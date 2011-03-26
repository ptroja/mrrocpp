#include "../irp6ot_m/ui_r_irp6ot_m.h"
#include "../irp6p_m/ui_r_irp6p_m.h"
#include "../conveyor/ui_r_conveyor.h"

#include "wgt_process_control.h"
#include "interface.h"

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

void wgt_process_control::my_open()
{
	wgt_base::my_open();
	process_control_window_init();
}

void wgt_process_control::on_mp_start_pushButton_clicked()
{
	interface.pulse_start_mp();
}

void wgt_process_control::on_mp_stop_pushButton_clicked()
{
	interface.pulse_stop_mp();
}

void wgt_process_control::on_mp_pause_pushButton_clicked()
{
	interface.pulse_pause_mp();
}

void wgt_process_control::on_mp_resume_pushButton_clicked()
{
	interface.pulse_resume_mp();
}

void wgt_process_control::on_mp_trigger_pushButton_clicked()
{
	interface.pulse_trigger_mp();
}

//ECP
void wgt_process_control::on_all_ecp_trigger_pushButton_clicked()
{
	interface.pulse_trigger_ecp();
}

// Reader
void wgt_process_control::on_all_reader_start_pushButton_clicked()
{
	interface.pulse_start_all_reader();
}

void wgt_process_control::on_all_reader_stop_pushButton_clicked()
{
	interface.pulse_stop_all_reader();
}

void wgt_process_control::on_all_reader_trigger_pushButton_clicked()
{
	interface.pulse_trigger_all_reader();
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

	// Dla irp6_on_track

	interface.irp6ot_m->process_control_window_irp6ot_section_init(wlacz_PtButton_wnd_processes_control_all_reader_start, wlacz_PtButton_wnd_processes_control_all_reader_stop, wlacz_PtButton_wnd_processes_control_all_reader_trigger);

	// Dla irp6_postument

	interface.irp6p_m->process_control_window_irp6p_section_init(wlacz_PtButton_wnd_processes_control_all_reader_start, wlacz_PtButton_wnd_processes_control_all_reader_stop, wlacz_PtButton_wnd_processes_control_all_reader_trigger);

	// Dla conveyor

	interface.conveyor->process_control_window_conveyor_section_init(wlacz_PtButton_wnd_processes_control_all_reader_start, wlacz_PtButton_wnd_processes_control_all_reader_stop, wlacz_PtButton_wnd_processes_control_all_reader_trigger);

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
	if (interface.mp.state != interface.mp.last_process_control_state) {
		switch (interface.mp.state)
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

		interface.mp.last_process_control_state = interface.mp.state;

	}

	return 1;

}

int wgt_process_control::block_all_ecp_trigger_widgets()

{

	/* TR

	 if (interface.irp6ot_m->state.edp.is_synchronised) {
	 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6ot_ecp_trigger);
	 }
	 if (interface.irp6p_m->state.edp.is_synchronised) {
	 interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_ecp_trigger);
	 }
	 if (interface.conveyor->state.edp.is_synchronised) {
	 interface.block_widget(ABW_PtButton_wnd_processes_control_conveyor_ecp_trigger);
	 }
	 */

	ui->all_ecp_trigger_pushButton->setDisabled(true);

	return 1;
}

int wgt_process_control::unblock_all_ecp_trigger_widgets()

{

	/* TR

	 if (interface.irp6ot_m->state.edp.is_synchronised) {
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6ot_ecp_trigger);
	 }
	 if (interface.irp6p_m->state.edp.is_synchronised) {
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6p_ecp_trigger);
	 }
	 if (interface.conveyor->state.edp.is_synchronised) {
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_conveyor_ecp_trigger);
	 }
	 */

	ui->all_ecp_trigger_pushButton->setDisabled(false);
	return 1;
}
