#include "wgt_process_control.h"
#include "interface.h"

wgt_process_control::wgt_process_control(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	wgt_base("Process control", _interface, parent)
{

	ui.setupUi(this);

	connect(this, SIGNAL(process_control_window_init_signal()), this, SLOT(process_control_window_init_slot()), Qt::QueuedConnection);

	init(true);
}

wgt_process_control::~wgt_process_control()
{

}

void wgt_process_control::process_control_window_init()
{
	emit process_control_window_init_signal();
}

void wgt_process_control::process_control_window_init_slot()
{
	init(false);
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
int wgt_process_control::init(bool do_it)

{
	/* TR


	 if (interface.is_process_control_window_open) {

	 bool wlacz_PtButton_wnd_processes_control_all_reader_start = false;
	 bool wlacz_PtButton_wnd_processes_control_all_reader_stop = false;
	 bool wlacz_PtButton_wnd_processes_control_all_reader_trigger = false;

	 // Dla READER'A

	 interface.block_widget(ABW_PtButton_wnd_processes_control_all_reader_start);
	 interface.block_widget(ABW_PtButton_wnd_processes_control_all_reader_stop);
	 interface.block_widget(ABW_PtButton_wnd_processes_control_all_reader_trigger);

	 // Dla irp6_on_track

	 interface.irp6ot_m->process_control_window_irp6ot_section_init(wlacz_PtButton_wnd_processes_control_all_reader_start, wlacz_PtButton_wnd_processes_control_all_reader_stop, wlacz_PtButton_wnd_processes_control_all_reader_trigger);

	 // Dla irp6_postument

	 interface.irp6p_m->process_control_window_irp6p_section_init(wlacz_PtButton_wnd_processes_control_all_reader_start, wlacz_PtButton_wnd_processes_control_all_reader_stop, wlacz_PtButton_wnd_processes_control_all_reader_trigger);

	 // Dla conveyor

	 interface.conveyor->process_control_window_conveyor_section_init(wlacz_PtButton_wnd_processes_control_all_reader_start, wlacz_PtButton_wnd_processes_control_all_reader_stop, wlacz_PtButton_wnd_processes_control_all_reader_trigger);

	 // Dla speakera - wylaczone

	 // Dla irp6_mechatronika


	 // All reader's pulse buttons
	 if (wlacz_PtButton_wnd_processes_control_all_reader_start) {
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_all_reader_start);
	 }

	 if (wlacz_PtButton_wnd_processes_control_all_reader_stop) {
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_all_reader_stop);
	 }

	 if (wlacz_PtButton_wnd_processes_control_all_reader_trigger) {
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_all_reader_trigger);
	 }

	 // Dla mp i ecp
	 if ((interface.mp.state != interface.mp.last_state) || (do_it)) {
	 interface.process_control_window_renew = false;

	 switch (interface.mp.state)
	 {
	 case ui::common::UI_MP_PERMITED_TO_RUN:
	 interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_start);
	 interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_stop);
	 interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_pause);
	 interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_resume);
	 interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_trigger);

	 block_all_ecp_trigger_widgets(NULL, NULL, NULL);
	 break;
	 case ui::common::UI_MP_WAITING_FOR_START_PULSE:
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_mp_pulse_start);
	 interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_stop);
	 interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_pause);
	 interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_resume);
	 interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_trigger);

	 block_all_ecp_trigger_widgets(NULL, NULL, NULL);
	 break;
	 case ui::common::UI_MP_TASK_RUNNING:
	 interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_start);
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_mp_pulse_stop);
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_mp_pulse_pause);
	 interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_resume);
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_mp_pulse_trigger);

	 unblock_all_ecp_trigger_widgets(NULL, NULL, NULL);
	 break;
	 case ui::common::UI_MP_TASK_PAUSED:
	 interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_start);
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_mp_pulse_stop);
	 interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_pause);
	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_mp_pulse_resume);
	 interface.block_widget(ABW_PtButton_wnd_processes_control_mp_pulse_trigger);

	 block_all_ecp_trigger_widgets(NULL, NULL, NULL);
	 break;
	 default:
	 break;
	 }

	 interface.mp.last_state = interface.mp.state;

	 }

	 PtDamageWidget(ABW_wnd_processes_control);
	 }
	 */
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

	 interface.block_widget(ABW_PtButton_wnd_processes_control_all_ecp_trigger);
	 */
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

	 interface.unblock_widget(ABW_PtButton_wnd_processes_control_all_ecp_trigger);
	 */
	return 1;
}
