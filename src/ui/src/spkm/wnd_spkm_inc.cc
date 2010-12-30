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
	common::WndBase(_interface, ABN_wnd_spkm_inc, ABI_wnd_spkm_inc), robot(_robot)
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

				/*
				 for (int i = 0; i < lib::irp6ot_m::NUM_OF_SERVOS; i++)
				 interface.irp6ot_m->desired_pos[i] = interface.irp6ot_m->current_pos[i];
				 */
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

}
} //namespace ui
} //namespace mrrocpp
