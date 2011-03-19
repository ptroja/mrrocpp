/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/spkm/ui_ecp_r_spkm.h"
#include "ui/src/spkm/ui_r_spkm.h"
#include "ui/src/spkm/wnd_spkm_inc.h"
#include "robot/spkm/const_spkm.h"
#include "ui/src/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

namespace mrrocpp {
namespace ui {
namespace spkm {

//
//
// KLASA WndInc
//
//


WndInc::WndInc(common::Interface& _interface, UiRobot& _robot) :
	common::WndBase(WND_SPKM_INC, _interface, ABN_wnd_spkm_inc, ABI_wnd_spkm_inc), robot(_robot)
{

}

int WndInc::init()
{

	try {

		if (robot.state.edp.pid != -1) {
			if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{

				interface.unblock_widget(ABW_PtPane_wind_spkm_inc_post_synchro_moves);

				robot.ui_ecp_robot->epos_reply_data_request_port->set_request();
				robot.ui_ecp_robot->execute_motion();
				robot.ui_ecp_robot->epos_reply_data_request_port->get();

				set_single_axis(0, ABW_PtNumericFloat_wind_spkm_motors_mcur_0, ABW_PtNumericFloat_wind_spkm_motors_cur_p0, ABW_thumb_wind_spkm_motors_mip_0);
				set_single_axis(1, ABW_PtNumericFloat_wind_spkm_motors_mcur_1, ABW_PtNumericFloat_wind_spkm_motors_cur_p1, ABW_thumb_wind_spkm_motors_mip_1);
				set_single_axis(2, ABW_PtNumericFloat_wind_spkm_motors_mcur_2, ABW_PtNumericFloat_wind_spkm_motors_cur_p2, ABW_thumb_wind_spkm_motors_mip_2);
				set_single_axis(3, ABW_PtNumericFloat_wind_spkm_motors_mcur_3, ABW_PtNumericFloat_wind_spkm_motors_cur_p3, ABW_thumb_wind_spkm_motors_mip_3);
				set_single_axis(4, ABW_PtNumericFloat_wind_spkm_motors_mcur_4, ABW_PtNumericFloat_wind_spkm_motors_cur_p4, ABW_thumb_wind_spkm_motors_mip_4);
				set_single_axis(5, ABW_PtNumericFloat_wind_spkm_motors_mcur_5, ABW_PtNumericFloat_wind_spkm_motors_cur_p5, ABW_thumb_wind_spkm_motors_mip_5);

				for (int i = 0; i < robot.number_of_servos; i++) {
					robot.desired_pos[i] = robot.current_pos[i];
				}

			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				interface.block_widget(ABW_PtPane_wind_spkm_inc_post_synchro_moves);
			}
		}

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int WndInc::set_single_axis(int axis, PtWidget_t *ABW_current, PtWidget_t *ABW_position, PtWidget_t *ABW_thumb)
{

	lib::epos::epos_reply &er = robot.ui_ecp_robot->epos_reply_data_request_port->data;

	PtSetResource(ABW_current, Pt_ARG_NUMERIC_VALUE, &er.epos_controller[axis].current, 0);
	PtSetResource(ABW_position, Pt_ARG_NUMERIC_VALUE, &er.epos_controller[axis].position, 0);

	if (er.epos_controller[axis].motion_in_progress) {
		interface.set_toggle_button(ABW_thumb);
	} else {
		interface.unset_toggle_button(ABW_thumb);
	}

	return 1;
}

int WndInc::import()
{

	char *tmp_ptgr, *tmp;
	double val;

	PtGetResource(ABW_PtText_input_console, Pt_ARG_TEXT_STRING, &tmp_ptgr, 0);
	tmp = new char[strlen(tmp_ptgr)];
	strcpy(tmp, tmp_ptgr);

	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p0, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p1, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p2, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p3, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p4, Pt_ARG_NUMERIC_VALUE, &val, 0);
	val = strtod(tmp, &tmp);
	PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p5, Pt_ARG_NUMERIC_VALUE, &val, 0);

	return 1;
}

int WndInc::exporto()
{

	char buffer[200];

	double *wektor[robot.number_of_servos];

	PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p0, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
	PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p1, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
	PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p2, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
	PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p3, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
	PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p4, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
	PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p5, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);

	sprintf(buffer, "edp_spkm INCREMENTAL POSITION\n %f %f %f %f %f %f", *wektor[0], *wektor[1], *wektor[2], *wektor[3], *wektor[4], *wektor[5]);

	interface.ui_msg->message(buffer);

	return 1;
}

int WndInc::copy()
{

	// wychwytania ew. bledow ECP::robot
	double *wektor_ptgr[robot.number_of_servos], wektor[robot.number_of_servos];

	if (robot.state.edp.pid != -1) {
		if (robot.state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{

			interface.unblock_widget(ABW_PtPane_wind_spkm_inc_post_synchro_moves);

			PtGetResource(ABW_PtNumericFloat_wind_spkm_motors_cur_p0, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[0], 0);
			PtGetResource(ABW_PtNumericFloat_wind_spkm_motors_cur_p1, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[1], 0);
			PtGetResource(ABW_PtNumericFloat_wind_spkm_motors_cur_p2, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[2], 0);
			PtGetResource(ABW_PtNumericFloat_wind_spkm_motors_cur_p3, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[3], 0);
			PtGetResource(ABW_PtNumericFloat_wind_spkm_motors_cur_p4, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[4], 0);
			PtGetResource(ABW_PtNumericFloat_wind_spkm_motors_cur_p5, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr[5], 0);

			for (int i = 0; i < robot.number_of_servos; i++) {
				wektor[i] = *wektor_ptgr[i];
			}

			PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p0, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
			PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p1, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
			PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p2, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
			PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p3, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
			PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p4, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
			PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p5, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);

		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			interface.block_widget(ABW_PtPane_wind_spkm_inc_post_synchro_moves);

		}
	}

	return 1;
}

int WndInc::motion(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{

	double *wektor[robot.number_of_servos];
	double *step1;

	// wychwytania ew. bledow ECP::robot
	try {

		if (robot.state.edp.pid != -1) {

			if (robot.state.edp.is_synchronised) {

				PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p0, Pt_ARG_NUMERIC_VALUE, &wektor[0], 0);
				PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p1, Pt_ARG_NUMERIC_VALUE, &wektor[1], 0);
				PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p2, Pt_ARG_NUMERIC_VALUE, &wektor[2], 0);
				PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p3, Pt_ARG_NUMERIC_VALUE, &wektor[3], 0);
				PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p4, Pt_ARG_NUMERIC_VALUE, &wektor[4], 0);
				PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_p5, Pt_ARG_NUMERIC_VALUE, &wektor[5], 0);

				for (int i = 0; i < robot.number_of_servos; i++) {
					robot.desired_pos[i] = *wektor[i];
				}
			} else {

				for (int i = 0; i < robot.number_of_servos; i++) {
					robot.desired_pos[i] = 0.0;
				}
			}

			PtGetResource(ABW_PtNumericFloat_wind_spkm_inc_step, Pt_ARG_NUMERIC_VALUE, &step1, 0);

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_0l) {
				robot.desired_pos[0] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_1l) {
				robot.desired_pos[1] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_2l) {
				robot.desired_pos[2] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_3l) {
				robot.desired_pos[3] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_4l) {
				robot.desired_pos[4] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_5l) {
				robot.desired_pos[5] -= (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_0r) {
				robot.desired_pos[0] += (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_1r) {
				robot.desired_pos[1] += (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_2r) {
				robot.desired_pos[2] += (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_3r) {
				robot.desired_pos[3] += (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_4r) {
				robot.desired_pos[4] += (*step1);
			} else

			if (ApName(ApWidget(cbinfo)) == ABN_PtButton_wind_spkm_inc_5r) {
				robot.desired_pos[5] += (*step1);
			}

			//	std::cout << "UI desired_pos[4]" << desired_pos[4] << std::endl;

			robot.ui_ecp_robot->move_motors(robot.desired_pos);

			if ((robot.state.edp.is_synchronised) && (is_open)) { // by Y o dziwo nie dziala poprawnie 	 if (robot.state.edp.is_synchronised)

				PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p0, Pt_ARG_NUMERIC_VALUE, &robot.desired_pos[0], 0);
				PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p1, Pt_ARG_NUMERIC_VALUE, &robot.desired_pos[1], 0);
				PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p2, Pt_ARG_NUMERIC_VALUE, &robot.desired_pos[2], 0);
				PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p3, Pt_ARG_NUMERIC_VALUE, &robot.desired_pos[3], 0);
				PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p4, Pt_ARG_NUMERIC_VALUE, &robot.desired_pos[4], 0);
				PtSetResource(ABW_PtNumericFloat_wind_spkm_inc_p5, Pt_ARG_NUMERIC_VALUE, &robot.desired_pos[5], 0);

			}
		} // end if (robot.state.edp.pid!=-1)
	} // end try

	CATCH_SECTION_UI

	return 1;
}

}
} //namespace ui
} //namespace mrrocpp
