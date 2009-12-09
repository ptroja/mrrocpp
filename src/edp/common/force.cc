/*	! \file src/edp/irp6s/force.cc
 * \brief WATKI SILOWE
 * Ostatnia modyfikacja: kwiecie� 2006*/

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
#include "edp/common/edp_irp6s_postument_track.h"

/********************************* GLOBALS **********************************/

// watek do komunikacji miedzy edp a vsp

namespace mrrocpp {
namespace edp {
namespace common {

void * irp6s_postument_track_effector::edp_vsp_thread_start(void* arg)
{
	return static_cast<irp6s_postument_track_effector*> (arg)->edp_vsp_thread(arg);
}

void * irp6s_postument_track_effector::edp_vsp_thread(void *arg)
{
#if !defined(USE_MESSIP_SRR)
	name_attach_t *edp_vsp_attach;
	uint64_t e; //!< kod bledu systemowego
	int vsp_caller; //!< by Y&W
	lib::VSP_EDP_message vsp_edp_command;
	lib::EDP_VSP_reply edp_vsp_reply;

	lib::set_thread_priority(pthread_self() , MAX_PRIORITY-4);

	//!< zarejestrowanie nazwy identyfikujacej serwer

	if ((edp_vsp_attach = name_attach(NULL, config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "edp_vsp_attach_point").c_str(),
	NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		e = errno;
		perror("Failed to attach EDP_VSP");
		vs->sr_msg->message(lib::SYSTEM_ERROR, e, "Failed to attach Effector Control Process");
	}
	long counter = 0;
	while (1) {
		vsp_caller = MsgReceive(edp_vsp_attach->chid, &vsp_edp_command, sizeof(vsp_edp_command), NULL);

		if (vsp_caller == -1) /*!Error condition, exit */
		{
			e = errno;
			perror("EDP_VSP: Receive from VSP failed");
			vs->sr_msg->message(lib::SYSTEM_ERROR, e, "EDP: Receive from VSP failed");
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

		/*!A QNX IO message received, reject */
		if (vsp_edp_command.hdr.type >= _IO_BASE && vsp_edp_command.hdr.type <= _IO_MAX) {
			MsgReply(vsp_caller, EOK, 0, 0);
			continue;
		}
		/*
		 if (vsp_edp_command.konfigurowac)
		 vs->force_sensor_do_configure = true;//!< jesli otrzymano od VSP polecenie konfiguracji czujnika
		 */
		//!< oczekiwanie nowego pomiaru
		sem_wait(&(vs->new_ms));
		//!< przygotowanie struktury do wyslania
		double current_force[6];

		lib::Homog_matrix current_frame_wo_offset = return_current_frame(WITHOUT_TRANSLATION);
		lib::Ft_v_tr ft_tr_inv_current_frame_matrix(!current_frame_wo_offset, lib::Ft_v_tr::FT);

		lib::Homog_matrix current_tool(get_current_kinematic_model()->tool);
		lib::Ft_v_tr ft_tr_inv_tool_matrix(!current_tool, lib::Ft_v_tr::FT);

		// uwaga sila nie przemnozona przez tool'a i current frame orientation
		force_msr_download(current_force, 0);

		lib::Ft_v_vector current_force_torque(ft_tr_inv_tool_matrix * ft_tr_inv_current_frame_matrix
				* lib::Ft_v_vector(current_force));
		current_force_torque.to_table(edp_vsp_reply.force);

		counter++;

		rb_obj->lock_mutex();
		edp_vsp_reply.servo_step=rb_obj->step_data.step;
		for (int i=0; i<=5; i++) {
			edp_vsp_reply.current_present_XYZ_ZYZ_arm_coordinates[i]=rb_obj->step_data.current_cartesian_position[i];
		}

		rb_obj->unlock_mutex();

		//!< wyslanie danych
		if (MsgReply(vsp_caller, EOK, &edp_vsp_reply, sizeof(edp_vsp_reply)) ==-1) //!< by Y&W
		{
			e = errno;
			perror("EDP_VSP: Reply to VSP failed");
			vs->sr_msg->message(lib::SYSTEM_ERROR, e, "EDP: Reply to VSP failed");
		}
	} //!< end while
#endif /* USE_MESSIP_SRR */
	return NULL;
}

void * irp6s_postument_track_effector::force_thread_start(void* arg)
{
	return static_cast<irp6s_postument_track_effector*> (arg)->force_thread(arg);
}

//!< watek do komunikacji ze sprzetem
void * irp6s_postument_track_effector::force_thread(void *arg)
{
#if !defined(USE_MESSIP_SRR)
	lib::set_thread_priority(pthread_self() , MAX_PRIORITY-1);

	vs = sensor::return_created_edp_force_sensor(*this); //!< czujnik wirtualny

	sem_post(&force_master_sem);

	try
	{
		vs->configure_sensor();
	}

	catch (lib::sensor::sensor_error e)
	{
		printf("sensor_error w force thread EDP\n");

		switch(e.error_no)
		{
			case SENSOR_NOT_CONFIGURED:
			vs->from_vsp.vsp_report= lib::VSP_SENSOR_NOT_CONFIGURED;
			break;
			case READING_NOT_READY:
			vs->from_vsp.vsp_report= lib::VSP_READING_NOT_READY;
			break;
		}
		vs->sr_msg->message (lib::FATAL_ERROR, e.error_no);

	} //!< end CATCH

	catch(...)
	{
		fprintf(stderr, "unidentified error force thread w EDP\n");
	}

	while (!vs->TERMINATE) //!< for (;;)
	{
		try
		{
			if (vs->force_sensor_do_first_configure)
			{ //!< jesli otrzymano polecenie konfiguracji czujnika
		//		printf("force_sensor_do_first_configure\n");
					vs->configure_sensor();
					vs->force_sensor_do_first_configure = false; //!< ustawienie flagi ze czujnik jest ponownie skonfigurowany
					vs->first_configure_done = true;
			}
			else if (vs->force_sensor_do_configure)
			{ //!< jesli otrzymano polecenie konfiguracji czujnika
				if (vs->new_edp_command)
				{
					vs->configure_sensor();
					vs->set_command_execution_finish();
					vs->force_sensor_do_configure = false; //!< ustawienie flagi ze czujnik jest ponownie skonfigurowany
				}
			}
			else if (vs->force_sensor_set_tool)
			{
				if (vs->new_edp_command)
				{
					vs->set_force_tool();
					vs->set_command_execution_finish();
					vs->force_sensor_set_tool = false;
				}
			}
			else
			{
				//!< cout << "przed Wait for event" << endl;
				vs->wait_for_event();
				//!< cout << "po Wait for event" << endl;

				vs->initiate_reading();
				//!< 		cout << "Initiate reading" << endl;

				double current_force[6];

				lib::Homog_matrix current_frame_wo_offset = return_current_frame(WITHOUT_TRANSLATION);
				lib::Ft_v_tr ft_tr_inv_current_frame_matrix (!current_frame_wo_offset, lib::Ft_v_tr::FT);

				lib::Homog_matrix current_tool(get_current_kinematic_model()->tool);
				lib::Ft_v_tr ft_tr_inv_tool_matrix (!current_tool, lib::Ft_v_tr::FT);

				// uwaga sila nie przemnozona przez tool'a i current frame orientation
				force_msr_download(current_force, 0);

				lib::Ft_v_vector current_force_torque (ft_tr_inv_tool_matrix * ft_tr_inv_current_frame_matrix * lib::Ft_v_vector (current_force));

				rb_obj->lock_mutex();
				for (int i=0;i<=5;i++)
				{
					current_force_torque.to_table (rb_obj->step_data.force);
				}
				rb_obj->unlock_mutex();
			}
			sem_trywait(&(vs->new_ms));
			sem_post(&(vs->new_ms)); //!< jest gotowy nowy pomiar

		} //!< koniec TRY

		catch (lib::sensor::sensor_error e)
		{
			printf("sensor_error w force thread  EDP\n");

			switch(e.error_no)
			{
				case SENSOR_NOT_CONFIGURED:
				vs->from_vsp.vsp_report= lib::VSP_SENSOR_NOT_CONFIGURED;
				break;
				case READING_NOT_READY:
				vs->from_vsp.vsp_report= lib::VSP_READING_NOT_READY;
				break;
			}
			vs->sr_msg->message (lib::FATAL_ERROR, e.error_no);
		} //!< end CATCH

		catch(...)
		{
			printf("unidentified error force thread w EDP\n");
		}

	} //!< //!< end while(;;)
#endif /* USE_MESSIP_SRR */
	return NULL;
} //!< end MAIN

} // namespace common
} // namespace edp
} // namespace mrrocpp

