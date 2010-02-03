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

//!< watek do komunikacji ze sprzetem
void force::operator()(void)
{
#if !defined(USE_MESSIP_SRR)
	lib::set_thread_priority(pthread_self(), MAX_PRIORITY - 1);

	connect_to_hardware();

	master.force_thread_started.command();

	try {
		configure_sensor();
	}

	catch (lib::sensor::sensor_error e) {
		printf("sensor_error w force thread EDP\n");

		switch (e.error_no)
		{
			case SENSOR_NOT_CONFIGURED:
				from_vsp.vsp_report = lib::VSP_SENSOR_NOT_CONFIGURED;
				break;
			case READING_NOT_READY:
				from_vsp.vsp_report = lib::VSP_READING_NOT_READY;
				break;
		}
		sr_msg->message(lib::FATAL_ERROR, e.error_no);

	} //!< end CATCH

	catch (...) {
		fprintf(stderr, "unidentified error force thread w EDP\n");
	}

	while (!TERMINATE) //!< for (;;)
	{
		try {
			if (new_edp_command) {
				boost::mutex::scoped_lock lock(mtx);
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
				//!< cout << "przed Wait for event" << endl;
				wait_for_event();
				//!< cout << "po Wait for event" << endl;

				initiate_reading();
				//!< 		cout << "Initiate reading" << endl;

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
					boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

					current_force_torque.to_table(master.rb_obj->step_data.force);
				}
			}
			edp_vsp_synchroniser.command();

		} //!< koniec TRY

		catch (lib::sensor::sensor_error e) {
			printf("sensor_error w force thread  EDP\n");

			switch (e.error_no)
			{
				case SENSOR_NOT_CONFIGURED:
					from_vsp.vsp_report = lib::VSP_SENSOR_NOT_CONFIGURED;
					break;
				case READING_NOT_READY:
					from_vsp.vsp_report = lib::VSP_READING_NOT_READY;
					break;
			}
			sr_msg->message(lib::FATAL_ERROR, e.error_no);
		} //!< end CATCH

		catch (...) {
			printf("unidentified error force thread w EDP\n");
		}

	} //!< //!< end while(;;)
#endif /* USE_MESSIP_SRR */
} //!< end MAIN

force::force(common::irp6s_postument_track_effector &_master) :
	gravity_transformation(NULL), new_edp_command(false), master(_master), edp_vsp_synchroniser(),
			new_command_synchroniser()
{
	gravity_transformation = NULL;
	is_sensor_configured = false; //!< czujnik niezainicjowany
	is_reading_ready = false; //!< nie ma zadnego gotowego odczytu
	TERMINATE = false;

	/*!Lokalizacja procesu wywietlania komunikatow SR */
	sr_msg
			= new lib::sr_vsp(lib::EDP, master.config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "edp_vsp_attach_point"), master.config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", UI_SECTION), true);
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

	for (int i = 0; i < 3; i++) {
		current_force_tool_position[i] = next_force_tool_position[i];
	}
	current_force_tool_weight = next_force_tool_weight;
}

int force::set_command_execution_finish() // podniesienie semafora
{

	new_edp_command = false;
	new_command_synchroniser.command();

	return 0; // TODO: check for return or throw in future object-oriented version
}

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

