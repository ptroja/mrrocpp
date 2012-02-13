/*!
 * @file
 * @brief File contains weight measure generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include <cstdio>
#include <fstream>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <cmath>
#include <iostream>

#include "base/lib/typedefs.h"

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_robot.h"
#include "generator/ecp/weight_measure/ecp_g_weight_measure.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			weight_measure_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////

weight_measure::weight_measure(common::task::task& _ecp_task, double _weight_difference, double _catch_time) :
		common::generator::generator(_ecp_task),
		weight_difference(_weight_difference),
		current_buffer_pointer(0),
		initial_weight(0.0),
		initial_weight_counted(false),
		catch_time(_catch_time),
		terminate_state_recognized(false)
{
	generator_name = ecp_mp::generator::ECP_GEN_WEIGHT_MEASURE;
	clear_buffer();
}

void weight_measure::insert_in_buffer(double fx)
{

	weight_in_cyclic_buffer[current_buffer_pointer] = fx;

	if ((++current_buffer_pointer) == WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE) {
		current_buffer_pointer = 0;
	}

}

void weight_measure::clear_buffer()
{
	for (int i = 0; i < WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE; i++) {
		weight_in_cyclic_buffer[current_buffer_pointer] = 0.0;
	}
	current_buffer_pointer = 0;
	initial_weight_counted = false;
	terminate_state_recognized = false;

	catch_lag = initial_catch_lag = (int) (1000000 * catch_time / (USLEEP_TIME));
	// std::cout << "weight_measure_generator" << initial_catch_lag << std::endl;

}

double weight_measure::check_average_weight_in_buffer(void) const
{
	double returned_value = 0.0;

	for (int i = 0; i < WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE; i++) {
		returned_value += weight_in_cyclic_buffer[current_buffer_pointer];
	}
	returned_value /= WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE;
	return returned_value;
}

void weight_measure::set_weight_difference(double _weight_difference)
{
	weight_difference = _weight_difference;
}

bool weight_measure::first_step()
{
	clear_buffer();

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
//	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.interpolation_type = lib::TCIM;

	return true;
}

bool weight_measure::next_step()
{
	usleep(USLEEP_TIME);

	if (check_and_null_trigger()) {
		return false;
	}

	// transformacja ciezaru do osi z ukladu bazowego
	lib::Homog_matrix current_frame_wo_offset(the_robot->reply_package.arm.pf_def.arm_frame);
	current_frame_wo_offset.remove_translation();

	//	std::cout << 	current_frame_wo_offset << std::endl;

	lib::Ft_v_vector force_torque(lib::Ft_tr(current_frame_wo_offset)
			* lib::Ft_vector(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz));

	insert_in_buffer(-force_torque[2]);

	//std::cout << 	-force_torque[2] << std::endl;

	// nie wyznaczono jeszcze wagi poczatkowej
	if (!initial_weight_counted) {
		if (current_buffer_pointer == 0) {
			initial_weight_counted = true;
			initial_weight = check_average_weight_in_buffer();
		}

		return true;
	} else
	//  wyznaczono wage poczatkowa
	{

		if (((weight_difference > 0) && (check_average_weight_in_buffer() - initial_weight) > weight_difference)
				|| ((weight_difference < 0) && (check_average_weight_in_buffer() - initial_weight) < weight_difference))

				{
			// wszytkie potwierdzenia warunku koncowego musza wystapic pod rzad
			if (!terminate_state_recognized) {
				catch_lag = initial_catch_lag;
			}

			terminate_state_recognized = true;
			//    	printf("check_average_weight_in_buffer: %f, %f\n", check_average_weight_in_buffer(), initial_weight );
			if ((--catch_lag) <= 0) {
				return false;
			} else {
				return true;
			}
		} else {
			terminate_state_recognized = false;
			return true;
		}
	}

	return true;
}

void weight_measure::conditional_execution()
{
	set_weight_difference(ecp_t.mp_command.ecp_next_state.sg_buf.get <double>());

	Move();
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
