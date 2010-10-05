#include <iostream>

#include "base/edp/edp_typedefs.h"
#include "base/edp/edp_e_manip.h"
#include "base/lib/mis_fun.h"
#include "base/edp/reader.h"
#include "base/kinematics/kinematic_model_with_tool.h"
#include "base/edp/edp_force_sensor.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

//!< watek do komunikacji ze sprzetem
void force::operator()()
{

	//	sr_msg->message("operator");

	lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 1);

	connect_to_hardware();

	thread_started.command();

	try {
		configure_sensor();
	}

	catch (lib::sensor::sensor_error & e) {
		std::cerr << "sensor_error w force thread EDP" << std::endl;

		switch (e.error_no)
		{
			case SENSOR_NOT_CONFIGURED:
				from_vsp.vsp_report = lib::sensor::VSP_SENSOR_NOT_CONFIGURED;
				break;
			case READING_NOT_READY:
				from_vsp.vsp_report = lib::sensor::VSP_READING_NOT_READY;
				break;
		}
		sr_msg->message(lib::FATAL_ERROR, e.error_no);

	} //!< end CATCH

	catch (...) {
		std::cerr << "unidentified error force thread w EDP" << std::endl;
	}
	//sr_msg->message("dupa 1");
	clock_gettime(CLOCK_MONOTONIC, &wake_time);

	while (!TERMINATE) //!< for (;;)
	{
		try {
			if (new_edp_command) {
				boost::mutex::scoped_lock lock(mtx);
				// TODO: this should be handled with boost::bind functor parameter
				switch (command)
				{
					case (common::FORCE_SET_TOOL):
						set_force_tool();
						break;
					case (common::FORCE_CONFIGURE):
						configure_sensor();
						break;
					default:
						break;
				}
				set_command_execution_finish();
			} else {

				//	sr_msg->message("else 12");
				wait_for_event();

				initiate_reading();

				get_reading();

				lib::Ft_vector current_force;

				lib::Homog_matrix current_frame_wo_offset = master.return_current_frame(common::WITHOUT_TRANSLATION);
				lib::Ft_tr ft_tr_inv_current_frame_matrix(!current_frame_wo_offset);

				lib::Homog_matrix
						current_tool(((mrrocpp::kinematics::common::kinematic_model_with_tool*) master.get_current_kinematic_model())->tool);
				lib::Ft_tr ft_tr_inv_tool_matrix(!current_tool);

				// uwaga sila nie przemnozona przez tool'a i current frame orientation
				master.force_msr_download(current_force);

				lib::Ft_vector current_force_torque(ft_tr_inv_tool_matrix * ft_tr_inv_current_frame_matrix
						* current_force);

				// scope-locked reader data update
				{
					if (master.rb_obj) {
						boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

						current_force_torque.to_table(master.rb_obj->step_data.force);
					} else {
						std::cerr << "Error reader object not ready (force::operator()())" << std::endl;
					}
				}
			}
			edp_vsp_synchroniser.command();

		} //!< koniec TRY

		catch (lib::sensor::sensor_error & e) {
			std::cerr << "sensor_error in EDP force thread" << std::endl;

			switch (e.error_no)
			{
				case SENSOR_NOT_CONFIGURED:
					from_vsp.vsp_report = lib::sensor::VSP_SENSOR_NOT_CONFIGURED;
					break;
				case READING_NOT_READY:
					from_vsp.vsp_report = lib::sensor::VSP_READING_NOT_READY;
					break;
			}
			sr_msg->message(lib::FATAL_ERROR, e.error_no);
		} //!< end CATCH

		catch (...) {
			std::cerr << "unidentified error in EDP force thread" << std::endl;
		}

	} //!< //!< end while(;;)
} //!< end MAIN

force::force(common::manip_effector &_master) :
			is_reading_ready(false), //!< nie ma zadnego gotowego odczytu
			is_right_turn_frame(true), gravity_transformation(NULL), master(_master), TERMINATE(false),
			is_sensor_configured(false), new_edp_command(false) //!< czujnik niezainicjowany
{
	/*!Lokalizacja procesu wywietlania komunikatow SR */
	sr_msg
			= new lib::sr_vsp(lib::EDP, master.config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "edp_vsp_attach_point"), master.config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", lib::UI_SECTION), true);

	sr_msg->message("force");

	if (master.config.exists("is_right_turn_frame")) {
		is_right_turn_frame = master.config.value <bool> ("is_right_turn_frame");
	}

}

force::~force()
{
	delete sr_msg;
}

void force::set_force_tool(void)
{
	lib::K_vector gravity_arm_in_sensor(next_force_tool_position);
	lib::Homog_matrix frame = master.return_current_frame(common::WITH_TRANSLATION);
	gravity_transformation->defineTool(frame, next_force_tool_weight, gravity_arm_in_sensor);

	current_force_tool_position = next_force_tool_position;
	current_force_tool_weight = next_force_tool_weight;
}

void force::set_command_execution_finish() // podniesienie semafora
{
	new_edp_command = false;
	new_command_synchroniser.command();
}

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
