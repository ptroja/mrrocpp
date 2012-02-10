#include <iostream>
#include <exception>

#include "edp_typedefs.h"
#include "edp_e_manip.h"
#include "base/lib/mis_fun.h"
#include "reader.h"
#include "base/kinematics/kinematic_model_with_tool.h"
#include "edp_force_sensor.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

//!< watek do komunikacji ze sprzetem
void force::operator()()
{
	//	sr_msg->message("operator");

	if(!master.robot_test_mode) {
		lib::set_thread_priority(lib::PTHREAD_MAX_PRIORITY - 1);
	}

	try {
		if (!force_sensor_test_mode) {
			connect_to_hardware();
		}

		thread_started.command();

		configure_sensor();
	}

	catch (lib::exception::se_sensor & error) {
		std::cerr << "sensor_error w force thread EDP" << std::endl;

		uint64_t error0 = 0;

		if (uint64_t const * tmp = boost::get_error_info <mrrocpp_error0>(error)) {
			error0 = *tmp;
		}

		switch (error0)
		{
			case SENSOR_NOT_CONFIGURED:
				from_vsp.vsp_report = lib::sensor::VSP_SENSOR_NOT_CONFIGURED;
				break;
			case READING_NOT_READY:
				from_vsp.vsp_report = lib::sensor::VSP_READING_NOT_READY;
				break;
		}
		sr_msg->message(lib::FATAL_ERROR, error0);

	}

	catch (std::exception & e) {
		printf("force sensor exception: %s\n", e.what());
		sr_msg->message(lib::FATAL_ERROR, e.what());
		exit(EXIT_SUCCESS);
	}

	catch (...) {
		std::cerr << "unidentified error force thread w EDP" << std::endl;
	}

	if (clock_gettime(CLOCK_MONOTONIC, &wake_time) == -1) {
		perror("clock_gettime()");
	}

	while (!boost::this_thread::interruption_requested())
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

				get_reading();

				lib::Ft_vector current_force;

				lib::Homog_matrix current_frame_wo_offset = master.return_current_frame(common::WITHOUT_TRANSLATION);
				lib::Ft_tr ft_tr_inv_current_frame_matrix(!current_frame_wo_offset);

				lib::Homog_matrix current_tool(((mrrocpp::kinematics::common::kinematic_model_with_tool*) master.get_current_kinematic_model())->tool);
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
						std::cerr << " " << std::endl;
					}
				}
			}
			edp_vsp_synchroniser.command();

		} //!< koniec TRY

		catch (lib::exception::se_sensor & error) {
			std::cerr << "sensor_error w force thread EDP" << std::endl;

			uint64_t error0 = 0;

			if (uint64_t const * tmp = boost::get_error_info <mrrocpp_error0>(error)) {
				error0 = *tmp;
			}

			switch (error0)
			{
				case SENSOR_NOT_CONFIGURED:
					from_vsp.vsp_report = lib::sensor::VSP_SENSOR_NOT_CONFIGURED;
					break;
				case READING_NOT_READY:
					from_vsp.vsp_report = lib::sensor::VSP_READING_NOT_READY;
					break;
			}
			sr_msg->message(lib::FATAL_ERROR, error0);

		}

		catch (...) {
			std::cerr << "unidentified error in EDP force thread" << std::endl;
		}
	}
} //!< end MAIN

force::force(common::manip_effector &_master) :
		force_sensor_test_mode(true), is_reading_ready(false), //!< nie ma zadnego gotowego odczytu
		is_right_turn_frame(true), gravity_transformation(NULL), master(_master), is_sensor_configured(false), new_edp_command(false), cb(FORCE_BUFFER_LENGHT) //!< czujnik niezainicjowany
{
	/*! Lokalizacja procesu wywietlania komunikatow SR */

	sr_msg =
			boost::shared_ptr <lib::sr_vsp>(new lib::sr_vsp(lib::EDP, "f_" + master.config.robot_name, master.config.get_sr_attach_point()));

	sr_msg->message("force");

	if (master.config.exists(lib::FORCE_SENSOR_TEST_MODE)) {
		force_sensor_test_mode = master.config.exists_and_true(lib::FORCE_SENSOR_TEST_MODE);
	}

	if (force_sensor_test_mode) {
		sr_msg->message("Force sensor test mode activated");
	}

	if (master.config.exists("is_right_turn_frame")) {
		is_right_turn_frame = master.config.value <bool>("is_right_turn_frame");
	}

	for (int i = 0; i < 6; ++i) {
		ft_table[i] = 0.0;
	}

	clear_cb();

}

void force::clear_cb()
{

	lib::Ft_vector zero_force;
	for (int i = 0; i < 6; ++i) {
		zero_force[i] = 0.0;
	}
	cb.clear();

	for (int i = 0; i < FORCE_BUFFER_LENGHT; i++) {
		cb.push_back(zero_force);
	}

}

void force::wait_for_event()
{
	if (!force_sensor_test_mode) {

		wait_for_particular_event();

	} else {
		usleep(1000);
	}
}

/***************************** odczyt z czujnika *****************************/
void force::get_reading(void)
{
	if (!is_sensor_configured) {
		BOOST_THROW_EXCEPTION(lib::exception::fe_sensor() << lib::exception::mrrocpp_error0(SENSOR_NOT_CONFIGURED));
	}

	is_reading_ready = true;

	if (!force_sensor_test_mode) {
		get_particular_reading();

		// jesli ma byc wykorzytstywana biblioteka transformacji sil
		if (gravity_transformation) {
			lib::Homog_matrix frame = master.return_current_frame(common::WITH_TRANSLATION);
			// lib::Homog_matrix frame(master.force_current_end_effector_frame);

			bool overforce = false;
			for (int i = 0; i < 6; i++) {
				if ((fabs(ft_table[i]) > force_constraints[i]) || (!(std::isfinite(ft_table[i])))) {
					overforce = true;

				}
			}
			/*
			 std::stringstream buffero(std::stringstream::in | std::stringstream::out);
			 buffero << "over_force detected step: " << master.step_counter << " ";
			 for (int i = 0; i < 6; i++) {
			 buffero << i << ": " << ft_table[i] << " ";
			 }

			 std::cout << buffero.str() << std::endl;
			 */
			if (!overforce) {
				lib::Ft_vector base_force = gravity_transformation->getForce(ft_table, frame);

				lib::Homog_matrix current_orientation(frame.return_with_with_removed_translation());

				//sily przechowujemy w zerowej orientacji bazowej

				//		lib::Ft_vector force_in_base_orientation(lib::Ft_tr(current_orientation) * base_force);

				//		cb.push_back(force_in_base_orientation);
				cb.push_back(base_force);

				lib::Ft_vector output_in_base;

				for (int j = 0; j < 6; j++) {
					output_in_base[j] = 0.0;
				}

				// usredniamy za FORCE_BUFFER_LENGHT pomiarow

				for (int i = 0; i < FORCE_BUFFER_LENGHT; i++) {

					for (int j = 0; j < 6; j++) {
						output_in_base[j] += (cb[i][j]) / (double) FORCE_BUFFER_LENGHT;
					}

				}

				//sila zwracamy w biezacej oritntacji

				//	lib::Ft_vector output(lib::Ft_tr(!current_orientation) * output_in_base);

				lib::Ft_vector output = output_in_base;

				master.force_msr_upload(output);
			} else {
				std::stringstream buffer(std::stringstream::in | std::stringstream::out);
				buffer << "over_force detected step: " << master.step_counter << " ";
				for (int i = 0; i < 6; i++) {
					buffer << i << ": " << ft_table[i] << " ";
				}

				sr_msg->message(lib::NON_FATAL_ERROR, buffer.str());
			}
		}

	} else {
		master.force_msr_upload(ft_table);
	}

}

/**************************** inicjacja czujnika ****************************/
void force::configure_sensor(void)
{ // by Y

	is_sensor_configured = true;
	//  printf("edp Sensor configured\n");
	sr_msg->message("edp Sensor configured");

	if (!force_sensor_test_mode) {
		configure_particular_sensor();
	}

	clear_cb();

	// polozenie kisci bez narzedzia wzgledem bazy
	lib::Homog_matrix frame = master.return_current_frame(common::WITH_TRANSLATION); // FORCE Transformation by Slawomir Bazant
	// lib::Homog_matrix frame(master.force_current_end_effector_frame); // pobranie aktualnej ramki
	if (!gravity_transformation) // nie powolano jeszcze obiektu
	{

		// zczytanie sil maksymalnych
		if (master.config.exists("force_constraints")) {
			char *tmp = strdup(master.config.value <std::string>("force_constraints").c_str());
			char* toDel = tmp;
			for (int i = 0; i < 6; i++) {
				force_constraints[i] = strtod(tmp, &tmp);
			}
			free(toDel);
			// std::cout<<sensor_frame<<std::endl;
		}

		lib::Xyz_Angle_Axis_vector tab;
		if (master.config.exists("sensor_in_wrist")) {
			char *tmp = strdup(master.config.value <std::string>("sensor_in_wrist").c_str());
			char* toDel = tmp;
			for (int i = 0; i < 6; i++) {
				tab[i] = strtod(tmp, &tmp);
			}
			sensor_frame = lib::Homog_matrix(tab);
			free(toDel);
			// std::cout<<sensor_frame<<std::endl;
		}

		// lib::Homog_matrix sensor_frame = lib::Homog_matrix(0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0.09);

		double weight = master.config.value <double>("weight");

		double point[3];
		char *tmp = strdup(master.config.value <std::string>("default_mass_center_in_wrist").c_str());
		char* toDel = tmp;
		for (int i = 0; i < 3; i++)
			point[i] = strtod(tmp, &tmp);
		free(toDel);
		// double point[3] = { master.config.value<double>("x_axis_arm"),
		//		master.config.value<double>("y_axis_arm"), master.config.return_double_value("z_axis_arm") };
		lib::K_vector pointofgravity(point);
		gravity_transformation =
				new lib::ForceTrans(force_sensor_name, frame, sensor_frame, weight, pointofgravity, is_right_turn_frame);
	} else {
		gravity_transformation->synchro(frame);
	}

}

force::~force()
{
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
