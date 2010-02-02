/*	! \file src/edp/irp6s/force.cc
 * \brief WATKI SILOWE
 * Ostatnia modyfikacja: kwiecie��� 2006*/

/********************************* INCLUDES *********************************/
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stddef.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <semaphore.h>
#include <fstream>
#if !defined(USE_MESSIP_SRR)
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <devctl.h>
#include <string.h>
#include <signal.h>
#include <process.h>
#include <sys/sched.h>
#include <sys/netmgr.h>
#endif /* USE_MESSIP_SRR */
#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "lib/mis_fun.h"
#include "edp/common/edp.h"
#include "edp/common/reader.h"
#include "edp/common/edp_irp6s_postument_track.h"
#include "edp/common/edp_vsp_t.h"
#include "kinematics/common/kinematic_model_with_tool.h"

/********************************* GLOBALS **********************************/

// watek do komunikacji miedzy edp a vsp

namespace mrrocpp {
namespace edp {
namespace common {


edp_vsp::edp_vsp(irp6s_postument_track_effector &_master) :
	master (_master)
{
}

void edp_vsp::operator()(void)
{
#if !defined(USE_MESSIP_SRR)
	name_attach_t *edp_vsp_attach;
	uint64_t e; //!< kod bledu systemowego
	int vsp_caller; //!< by Y&W
	lib::VSP_EDP_message vsp_edp_command;
	lib::EDP_VSP_reply edp_vsp_reply;

	lib::set_thread_priority(pthread_self() , MAX_PRIORITY-4);

	//!< zarejestrowanie nazwy identyfikujacej serwer

	if ((edp_vsp_attach = name_attach(NULL, master.config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "edp_vsp_attach_point").c_str(),
	NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		e = errno;
		perror("Failed to attach EDP_VSP");
		master.vs->sr_msg->message(lib::SYSTEM_ERROR, e, "Failed to attach Effector Control Process");
	}
	long counter = 0;
	while (1) {
		vsp_caller = MsgReceive(edp_vsp_attach->chid, &vsp_edp_command, sizeof(vsp_edp_command), NULL);

		if (vsp_caller == -1) /*!Error condition, exit */
		{
			e = errno;
			perror("EDP_VSP: Receive from VSP failed");
			master.vs->sr_msg->message(lib::SYSTEM_ERROR, e, "EDP: Receive from VSP failed");
			break;
		}

		if (vsp_caller == 0) /*!Pulse received */
		{
			switch (vsp_edp_command.hdr.code) {
				case _PULSE_CODE_DISCONNECT:
					ConnectDetach(vsp_edp_command.hdr.scoid);
					break;
				case _PULSE_CODE_UNBLOCK:
					break;
				default:
					break;
			}
			continue;
		}

		/*! A QNX IO message received, reject */
		if (vsp_edp_command.hdr.type >= _IO_BASE && vsp_edp_command.hdr.type <= _IO_MAX) {
			MsgReply(vsp_caller, EOK, 0, 0);
			continue;
		}
		/*
		 if (vsp_edp_command.konfigurowac)
		 vs->force_sensor_do_configure = true;//!< jesli otrzymano od VSP polecenie konfiguracji czujnika
		 */
		//!< oczekiwanie nowego pomiaru
		master.vs->edp_vsp_synchroniser.wait();
		//!< przygotowanie struktury do wyslania
		lib::Ft_vector current_force;

		lib::Homog_matrix current_frame_wo_offset = master.return_current_frame(WITHOUT_TRANSLATION);
		lib::Ft_tr ft_tr_inv_current_frame_matrix(!current_frame_wo_offset);

		lib::Homog_matrix current_tool(((mrrocpp::kinematics::common::kinematic_model_with_tool*)master.get_current_kinematic_model())->tool);
		lib::Ft_tr ft_tr_inv_tool_matrix(!current_tool);

		// uwaga sila nie przemnozona przez tool'a i current frame orientation
		master.force_msr_download(current_force);

		lib::Ft_v_vector current_force_torque(ft_tr_inv_tool_matrix * ft_tr_inv_current_frame_matrix
				* current_force);
		current_force_torque.to_table(edp_vsp_reply.force);

		counter++;

    	// scope-locked reader data update
    	{
    		boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

			edp_vsp_reply.servo_step=master.rb_obj->step_data.step;
			for (int i=0; i<=5; i++) {
				edp_vsp_reply.current_present_XYZ_ZYZ_arm_coordinates[i]=master.rb_obj->step_data.desired_cartesian_position[i];
			}
    	}

		//!< wyslanie danych
		if (MsgReply(vsp_caller, EOK, &edp_vsp_reply, sizeof(edp_vsp_reply)) ==-1) //!< by Y&W
		{
			e = errno;
			perror("EDP_VSP: Reply to VSP failed");
			master.vs->sr_msg->message(lib::SYSTEM_ERROR, e, "EDP: Reply to VSP failed");
		}
	} //!< end while
#endif /* USE_MESSIP_SRR */
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

