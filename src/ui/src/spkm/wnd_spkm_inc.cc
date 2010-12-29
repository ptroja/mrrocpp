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
	common::WndBase(_interface, ABW_wnd_spkm_inc), robot(_robot)
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

				lib::epos::epos_reply &er = robot.ui_ecp_robot->epos_reply_data_request_port->data;

				if (er.epos_controller[0].motion_in_progress) {
					interface.set_toggle_button(ABW_thumb_wind_spkm_motors_mip_0);
				} else {
					interface.unset_toggle_button(ABW_thumb_wind_spkm_motors_mip_0);
				}

				/*
				 for (int i = 0; i < lib::irp6ot_m::NUM_OF_SERVOS; i++)
				 interface.irp6ot_m->desired_pos[i] = interface.irp6ot_m->current_pos[i];
				 */
			} else {
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				interface.block_widget(ABW_PtPane_wind_spkm_inc_post_synchro_moves);
			}
		}

		//	mrrocpp::lib::bird_hand::status &bhsrs = bird_hand.ui_ecp_robot->bird_hand_status_reply_data_request_port->data;


		/*
		 set_thumb_f_0_status();
		 set_thumb_f_1_status();

		 set_index_f_0_status();
		 set_index_f_1_status();
		 set_index_f_2_status();

		 set_ring_f_0_status();
		 set_ring_f_1_status();
		 set_ring_f_2_status();
		 */
	} // end try
	CATCH_SECTION_UI
	/*
	 interface.set_toggle_button(ABW_thumb_wind_spkm_motors_mip_0);
	 interface.set_toggle_button(ABW_thumb_wind_spkm_motors_mip_1);
	 interface.unset_toggle_button(ABW_thumb_wind_spkm_motors_mip_1);
	 */
	return 1;
}

}
} //namespace ui
} //namespace mrrocpp
