// ------------------------------------------------------------------------
//                                  edp.cc
//
// EDP_MASTER Effector Driver Master Process
// Driver dla robota IRp-6 na torze - metody: class edp_irp6s_robot
//
// Ostatnia modyfikacja: styczen 2005
// -------------------------------------------------------------------------

#include <stdio.h>
#include <semaphore.h>

#include "edp/common/edp.h"
#include "edp/common/edp_irp6s_postument_track.h"
#include "lib/mis_fun.h"
#include "edp/common/reader.h"
#include "kinematics/common/kinematic_model_with_tool.h"

namespace mrrocpp {
namespace edp {
namespace sensor {




void * force::thread_start(void* arg)
{
	return static_cast<force*> (arg)->thread_main_loop(arg);
}

void force::create_thread(void)
{
	if (pthread_create (&thread_id, NULL, &thread_start, (void *) this))
	{
	    master.msg->message(lib::SYSTEM_ERROR, errno, "EDP: Failed to create force thread");
	    throw common::System_error();
	}
}



//!< watek do komunikacji ze sprzetem
void * force::thread_main_loop(void *arg)
{
#if !defined(USE_MESSIP_SRR)
	lib::set_thread_priority(pthread_self() , MAX_PRIORITY-1);

	connect_to_hardware();

	sem_post(&(master.force_master_sem));

	try
	{
		configure_sensor();
	}

	catch (lib::sensor::sensor_error e)
	{
		printf("sensor_error w force thread EDP\n");

		switch(e.error_no)
		{
			case SENSOR_NOT_CONFIGURED:
			from_vsp.vsp_report= lib::VSP_SENSOR_NOT_CONFIGURED;
			break;
			case READING_NOT_READY:
			from_vsp.vsp_report= lib::VSP_READING_NOT_READY;
			break;
		}
		sr_msg->message (lib::FATAL_ERROR, e.error_no);

	} //!< end CATCH

	catch(...)
	{
		fprintf(stderr, "unidentified error force thread w EDP\n");
	}

	while (!TERMINATE) //!< for (;;)
	{
		try
		{
			if (force_sensor_do_first_configure)
			{ //!< jesli otrzymano polecenie konfiguracji czujnika
		//		printf("force_sensor_do_first_configure\n");
					configure_sensor();
					force_sensor_do_first_configure = false; //!< ustawienie flagi ze czujnik jest ponownie skonfigurowany
					first_configure_done = true;
			}
			else if (force_sensor_do_configure)
			{ //!< jesli otrzymano polecenie konfiguracji czujnika
				if (new_edp_command)
				{
					configure_sensor();
					set_command_execution_finish();
					force_sensor_do_configure = false; //!< ustawienie flagi ze czujnik jest ponownie skonfigurowany
				}
			}
			else if (force_sensor_set_tool)
			{
				if (new_edp_command)
				{
					set_force_tool();
					set_command_execution_finish();
					force_sensor_set_tool = false;
				}
			}
			else
			{
				//!< cout << "przed Wait for event" << endl;
				wait_for_event();
				//!< cout << "po Wait for event" << endl;

				initiate_reading();
				//!< 		cout << "Initiate reading" << endl;

				double current_force[6];

				lib::Homog_matrix current_frame_wo_offset = master.return_current_frame(common::WITHOUT_TRANSLATION);
				lib::Ft_tr ft_tr_inv_current_frame_matrix (!current_frame_wo_offset);

				lib::Homog_matrix current_tool(((mrrocpp::kinematics::common::kinematic_model_with_tool*)master.get_current_kinematic_model())->tool);
				lib::Ft_tr ft_tr_inv_tool_matrix (!current_tool);

				// uwaga sila nie przemnozona przez tool'a i current frame orientation
				master.force_msr_download(current_force, 0);

				lib::Ft_vector current_force_torque (ft_tr_inv_tool_matrix * ft_tr_inv_current_frame_matrix * lib::Ft_vector (current_force));

		    	// scope-locked reader data update
		    	{
		    		boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

								current_force_torque.to_table (master.rb_obj->step_data.force);
				}
			}
			sem_trywait(&(new_ms));
			sem_post(&(new_ms)); //!< jest gotowy nowy pomiar

		} //!< koniec TRY

		catch (lib::sensor::sensor_error e)
		{
			printf("sensor_error w force thread  EDP\n");

			switch(e.error_no)
			{
				case SENSOR_NOT_CONFIGURED:
				from_vsp.vsp_report= lib::VSP_SENSOR_NOT_CONFIGURED;
				break;
				case READING_NOT_READY:
				from_vsp.vsp_report= lib::VSP_READING_NOT_READY;
				break;
			}
			sr_msg->message (lib::FATAL_ERROR, e.error_no);
		} //!< end CATCH

		catch(...)
		{
			printf("unidentified error force thread w EDP\n");
		}

	} //!< //!< end while(;;)
#endif /* USE_MESSIP_SRR */
	return NULL;
} //!< end MAIN

force::force(common::irp6s_postument_track_effector &_master)
        : edp_extension_thread(_master), new_edp_command(false), master(_master)
{
    gravity_transformation = NULL;
    is_sensor_configured=false;	//!< czujnik niezainicjowany
    first_configure_done=false;
    is_reading_ready=false;				//!< nie ma zadnego gotowego odczytu
    force_sensor_do_first_configure = false;
    force_sensor_do_configure = false;
    force_sensor_set_tool = false;
    TERMINATE = false;

    sem_init( &new_ms, 0, 0);
    sem_init( &new_ms_for_edp, 0, 0);

    /*!Lokalizacja procesu wywietlania komunikatow SR */
    sr_msg = new lib::sr_vsp(lib::EDP, master.config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "edp_vsp_attach_point").c_str(),
                                 master.config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", UI_SECTION).c_str());
}

void force::set_force_tool(void)
{

    lib::K_vector gravity_arm_in_sensor(next_force_tool_position);
    lib::Homog_matrix frame = master.return_current_frame(common::WITH_TRANSLATION);
    gravity_transformation->defineTool(frame, next_force_tool_weight, gravity_arm_in_sensor);

    for (int i = 0; i<3; i++)
    {
        current_force_tool_position[i] = next_force_tool_position[i];
    }
    current_force_tool_weight = next_force_tool_weight;
}

int	force::set_command_execution_finish() // podniesienie semafora
{
    if (new_edp_command)
    {
        new_edp_command = false;
        sem_trywait(&(new_ms_for_edp));
        return sem_post(&new_ms_for_edp);// odwieszenie watku edp_master
    }

    return 0; // TODO: check for return or throw in future object-oriented version
}

int	force::check_for_command_execution_finish() // oczekiwanie na semafor
{
    new_edp_command = true;
    return sem_wait(&new_ms_for_edp);
}

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

