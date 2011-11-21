/*
 * ecp_t_objectfollower_pb_eih.cc
 *
 *  Created on: Apr 21, 2010
 *      Author: mboryn
 */

#include <stdexcept>

#include "ecp_t_pid_tuning_pb_eih.h"

#include "../defines.h"

#ifdef ROBOT_P
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#endif

#ifdef ROBOT_OT
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#endif

#include "../ecp_mp_g_visual_servo_tester.h"

using namespace std;
using namespace mrrocpp::ecp::common::generator;
using namespace logger;
using mrrocpp::ecp_mp::sensor::discode::discode_sensor;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

ecp_t_objectfollower_pb_eih::ecp_t_objectfollower_pb_eih(mrrocpp::lib::configurator& config) :
	common::task::task(config)
{
	try{
#ifdef ROBOT_P
		ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6p_m::robot(*this);
#endif
#ifdef ROBOT_OT
		ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6ot_m::robot(*this);
#endif

		newsmooth_gen = boost::shared_ptr<generator::newsmooth>(new generator::newsmooth(*this, lib::ECP_XYZ_ANGLE_AXIS, 6));

		char config_section_name[] = { "[pid_tuning_pb_eih]" };

		k_p_min = config.value<double>("k_p_min", config_section_name);
		k_p_max = config.value<double>("k_p_max", config_section_name);
		k_p_step = config.value<double>("k_p_step", config_section_name);
		step_distance = config.value<double>("step_distance", config_section_name);
		current_axis = config.value<int>("current_axis", config_section_name);

		regulator_axis = config.value<int>("regulator_axis", config_section_name);

		log_enabled = true;
		log_dbg_enabled = true;

		shared_ptr <position_constraint> cube(new cubic_constraint(config, config_section_name));

		reg = shared_ptr <regulator_p> (new regulator_p(config, config_section_name));

		sr_ecp_msg->message("Creating DisCODe sensor");
		boost::shared_ptr <discode_sensor> ds = boost::shared_ptr <discode_sensor>(new discode_sensor(config, config_section_name));
		vs = shared_ptr <visual_servo> (new pb_eih_visual_servo(reg, ds, config_section_name, config));

		obj_reached_term_cond = shared_ptr <object_reached_termination_condition> (new object_reached_termination_condition(config, config_section_name));
		timeout_term_cond = shared_ptr <timeout_termination_condition> (new timeout_termination_condition(config.value<double>("vs_timeout", config_section_name)));

		log_dbg("ecp_t_objectfollower_pb::ecp_t_objectfollower_pb(): 3\n");
		sm = shared_ptr <single_visual_servo_manager> (new single_visual_servo_manager(*this, config_section_name, vs));
		log_dbg("ecp_t_objectfollower_pb::ecp_t_objectfollower_pb(): 4\n");
		sm->add_position_constraint(cube);

		sm->add_termination_condition(obj_reached_term_cond);
		sm->add_termination_condition(timeout_term_cond);
		log_dbg("ecp_t_objectfollower_pb: configuring visual_servo_manager\n");
		sm->configure();
	}catch(exception& ex){
		sr_ecp_msg->message(lib::FATAL_ERROR, string("ERROR in ecp_t_objectfollower_pb_eih: ") + ex.what());
		throw ex;
	}
	log_dbg("ecp_t_objectfollower_pb: initialization completed.\n");
}

void ecp_t_objectfollower_pb_eih::main_task_algorithm(void)
{
//	newsmooth_gen->set_debug(true);
	while (1) {
		get_next_state();
		if (mp_2_ecp_next_state_string == mrrocpp::ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST) {
			// dojechac do przedmiotu
			sr_ecp_msg->message("Approaching the goal...");

			sm->Move();

			if(timeout_term_cond->is_condition_met()){
				sr_ecp_msg->message("Timeout. Object not found.");
				break;
			}

			if(!obj_reached_term_cond->is_condition_met()){
				sr_ecp_msg->message("Error: Object not reached.");
				break;
			}

			// zapamietac pozycje KR jako object_reached_position
			sr_ecp_msg->message("Saving object_reached_position");
			lib::Homog_matrix object_reached_position_hm = sm->get_current_position();

			lib::Xyz_Angle_Axis_vector xyz_aa;
			object_reached_position_hm.get_xyz_angle_axis(xyz_aa);
			xyz_aa.to_vector(object_reached_position);

			lib::Homog_matrix starting_position_hm = object_reached_position_hm;
			starting_position_hm(current_axis, 3) += step_distance;
			starting_position_hm.get_xyz_angle_axis(xyz_aa);
			xyz_aa.to_vector(starting_position);

			// dla wzmocnienia k_p_min ... k_p_max co krok k_p_step dla osi current_axis
			for(double k_p = k_p_min; k_p <= k_p_max; k_p += k_p_step){
				log("running with k_p = %g", k_p);

				reg->Kp(regulator_axis, regulator_axis) = k_p;

				// przesun KR do object_reached_position
				newsmooth_gen->reset();
				newsmooth_gen->load_absolute_angle_axis_trajectory_pose(object_reached_position);
				newsmooth_gen->calculate_interpolate();
				newsmooth_gen->Move();

				// przesun KR wzdluz osi current_axis o odleglosc step_distance
				newsmooth_gen->reset();
				newsmooth_gen->load_absolute_angle_axis_trajectory_pose(starting_position);
				newsmooth_gen->calculate_interpolate();
				newsmooth_gen->Move();

				// ustaw wzmocnienie k_p

				// uruchom VS z timeout_termination_condition i object_reached_termination_condition
				sm->Move();

				if(timeout_term_cond->is_condition_met()){
					// jesli warunek timeout_termination_condition zostal spelniony
					// zakoncz
					break;
				} else if (obj_reached_term_cond->is_condition_met()){
					// jesli warunek object_reached_termination_condition zostal spelniony
					// przesun KR do object_reached_position
				} else {
					// cos nie tak
				}
			}
			sr_ecp_msg->message("Finished");
		} else {
			log("ecp_t_objectfollower_pb::main_task_algorithm(void) mp_2_ecp_next_state_string: \"%s\"\n", mp_2_ecp_next_state_string.c_str());
		}
	}

	termination_notice();
}

task_base* return_created_ecp_task(lib::configurator &config)
{
	return new ecp_t_objectfollower_pb_eih(config);
}

} // namespace task

}//namespace common

}//namespace ecp

}//namespace mrrocpp
