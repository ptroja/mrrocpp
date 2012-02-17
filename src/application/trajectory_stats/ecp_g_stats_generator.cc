/**
 * @file ecp_g_stats_generator.cc
 * @brief Header file for stats_generator class
 * @author Tomasz Bem (mebmot@wp.pl)
 * @ingroup stats
 * @date 02.07.2010
 */

#include <ctime>

#include <boost/lexical_cast.hpp>

#include "base/ecp/ecp_robot.h"
#include "base/ecp/ecp_task.h"

#include "ecp_g_stats_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

const double current_ref[] = { 15000.0, 18000.0, 10000.0, 10000.0, 10000.0, 10000.0 };

/*================================Constructor=============================*//**
 * @brief Constructor along with task configurator.
 * @param _ecp_task Reference to task configurator.
 */
stats_generator::stats_generator(common::task::task& _ecp_task) :
		common::generator::generator(_ecp_task)
{
	reset();
}

/*==============================Destructor================================*//**
 * @brief Destructor.
 */
stats_generator::~stats_generator()
{
}

/*===============================first_step===============================*//**
 * @brief First step of stats generator
 * @details Initializes instruction for robot
 */
bool stats_generator::first_step()
{
	sr_ecp_msg.message("stats generator first step");
	printf("stats generator first step\n");
	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.set_arm_type = lib::FRAME;
//	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.interpolation_type = lib::MIM;
	the_robot->ecp_command.motion_steps = 10;
	the_robot->ecp_command.value_in_step_no = 10 - 2;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;

	mstep_ = 1;

	log_file_ = fopen(logFileName.c_str(), "w");

	return true;
}
/*================================next_step===============================*//**
 * @brief next step for stats generator.
 */
bool stats_generator::next_step()
{
	lib::Xyz_Angle_Axis_vector msr_position;
	lib::Homog_matrix position_matrix;
	double current_sum = 0;
	double current_norm = 0;

	the_robot->ecp_command.instruction_type = lib::SET_GET;
	printf("next step %d\n", mstep_);
	if (mstep_ > trj_.size()) {
		fclose(log_file_);
		return false;
	}

	the_robot->reply_package.arm.pf_def.arm_frame.get_xyz_angle_axis(msr_position);

	position_matrix.set_from_xyz_angle_axis(trj_[mstep_ - 1]);
	//send new position to the robot
	the_robot->ecp_command.arm.pf_def.arm_frame = position_matrix;

	for (int i = 0; i < 6; i++) {
		current_sum += the_robot->reply_package.arm.measured_current.average_module[i];

		double ref3 = current_ref[i] * current_ref[i] * current_ref[i];

		current_norm += the_robot->reply_package.arm.measured_current.average_cubic[i] / ref3;
	}

	current_sum /= 3;
	current_norm /= 3;

	fprintf(log_file_, "%f|%f|%f|%f|%f|%f|%f|%f\n", trj_[mstep_ - 1][0], trj_[mstep_ - 1][1], trj_[mstep_ - 1][2], msr_position[0], msr_position[1], msr_position[2], current_sum, current_norm);

	mstep_++;

	return true;
}

/**
 * @brief Resets generator between consecutive call of Move() method.
 * @details Resets all of the temporary variables. It is necessary to call the
 * reset between the calls of the generator Move() method.
 */
void stats_generator::reset()
{

}

mrrocpp::lib::Xyz_Angle_Axis_vector stats_generator::getFirstPosition()
{
	return trj_.front();
}

void stats_generator::load_trajectory(const std::string &filename)
{
	FILE * trj_file;

	trj_file = fopen(filename.c_str(), "r");

	if (trj_file > 0) {

		char line[100];

		while (!feof(trj_file)) {
			char * ret;

			ret = fgets(line, 100, trj_file);

			if (ret != NULL) {
				if (ret[0] == '\n' || ret[0] == '#' || ret[0] == '<') {
					continue;
				} else {
					double position[6];

					char * token = strtok(line, "|");

					if (token == NULL)
						continue;
					position[0] = boost::lexical_cast <double>(token);

					token = strtok(NULL, "|");

					if (token == NULL)
						continue;
					position[1] = boost::lexical_cast <double>(token);

					token = strtok(NULL, "\n");

					if (token == NULL)
						continue;
					position[2] = boost::lexical_cast <double>(token);

					position[3] = 1.203;
					position[4] = -1.447;
					position[5] = -0.294;

					/*token = strtok(NULL, "|");

					 if(token == NULL)
					 continue;
					 position[3] = boost::lexical_cast<double>(token);

					 token = strtok(NULL, "|");

					 if(token == NULL)
					 continue;
					 position[4] = boost::lexical_cast<double>(token);

					 token = strtok(NULL, "|");

					 if(token == NULL)
					 continue;
					 position[5] = boost::lexical_cast<double>(token);*/

					trj_.push_back(lib::Xyz_Angle_Axis_vector(position));

				}

			}
		}
	} else {

	}

	logFileName = filename + ".stat";
}

} //generator
} //common
} //ecp
} //mrrocpp
