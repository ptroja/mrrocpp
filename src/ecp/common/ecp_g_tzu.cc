#include <stdio.h>
#include <fstream>
#include <iostream>
#include <time.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_g_tzu.h"

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			force_meassure_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////


force_meassure_generator::force_meassure_generator(ecp_task& _ecp_task,
		double _weight_difference, double _catch_time, int _what_to_meassure) :
	ecp_generator(_ecp_task), weight_difference(_weight_difference),
			current_buffer_pointer(0), initial_weight(0.0),
			initial_weight_counted(false), 
			terminate_state_recognized(false), catch_time(_catch_time)
{
	what_to_meassure = _what_to_meassure;
	clear_buffer();

}

void force_meassure_generator::insert_in_buffer(double fx)
{

	weight_in_cyclic_buffer[current_buffer_pointer] = fx;

	if ((++current_buffer_pointer)==WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE)
	{
		current_buffer_pointer=0;
	}

}

void force_meassure_generator::clear_buffer()
{
	for (int i=0; i<WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE; i++)
	{
		weight_in_cyclic_buffer[current_buffer_pointer] = 0.0;
	}
	current_buffer_pointer=0;
	initial_weight_counted = false;
	terminate_state_recognized = false;
	
	catch_lag = initial_catch_lag = (int) 1000000*catch_time/(USLEEP_TIME);
//	std::cout << "weight_meassure_generator" << initial_catch_lag << std::endl;
	
}

double force_meassure_generator::check_average_weight_in_buffer(void) const
{
	double returned_value=0.0;

	for (int i=0; i<WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE; i++)
	{
		returned_value += weight_in_cyclic_buffer[current_buffer_pointer];
	}
	returned_value/=10;
	return returned_value;
}

void force_meassure_generator::set_weight_difference(double _weight_difference)
{
	weight_difference = _weight_difference;
}

bool force_meassure_generator::first_step()
{
	clear_buffer();

	the_robot->EDP_data.instruction_type = GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.get_arm_type = FRAME;
	the_robot->EDP_data.next_interpolation_type
			= EXTERNAL_INTERPOLATION_WITH_FORCE;

	return true;
}

double force_meassure_generator::get_meassurement()
{
	return check_average_weight_in_buffer();
}

void force_meassure_generator::change_meassurement(int what)
{
	what_to_meassure = what;
	clear_buffer();	
}

bool force_meassure_generator::next_step()
{
//	std::cout << "next step" << std::endl;
	usleep(USLEEP_TIME);
	
	if (check_and_null_trigger())
	{
		return false;
	}
	// transformacja ciezaru do osi z ukladu bazowego
	Homog_matrix current_frame_wo_offset(the_robot->EDP_data.current_arm_frame);
	current_frame_wo_offset.remove_translation();

	//	std::cout <<"frame: " <<	current_frame_wo_offset << std::endl;


	Ft_v_vector force_torque(Ft_v_tr(current_frame_wo_offset, Ft_v_tr::FT)
			* Ft_v_vector(the_robot->EDP_data.current_force_xyz_torque_xyz));

	insert_in_buffer(-force_torque[what_to_meassure]);

//	std::cout <<"torque0: "<< 	force_torque[0] << std::endl;
//	std::cout <<"torque1: "<< 	force_torque[1] << std::endl;
//	std::cout <<"torque2: "<< 	force_torque[2] << std::endl;
//	std::cout <<"torque3: "<< 	force_torque[3] << std::endl;
//	std::cout <<"torque4: "<< 	force_torque[4] << std::endl;
//	std::cout <<"torque5: "<< 	force_torque[5] << std::endl;

	// nie wyznaczono jeszcze wagi poczatkowej
	if (!initial_weight_counted)
	{
//		std::cout << "nie wyznaczono jeszcze wagi poczatkowej" << std::endl;
//		std::cout << "nie: " << current_buffer_pointer << std::endl;
		if (current_buffer_pointer==0)
		{
//			std::cout << "ustawiamy" << std::endl;
			initial_weight_counted = true;
			initial_weight = check_average_weight_in_buffer();
		}

		return true;
	}
	else
	//  wyznaczono wage poczatkowa
	{
//		std::cout << "wyznaczono wage poczatkowa" << std::endl;
//		std::cout << "catch lag bez zmian: " << catch_lag << std::endl;
//		std::cout << "weight_difference: " << weight_difference << std::endl;
//		std::cout << "check_average_weight_in_buffer(): " << check_average_weight_in_buffer() << std::endl;
//		std::cout << "check_average_weight_in_buffer() - initial_weight: " << check_average_weight_in_buffer() - initial_weight << std::endl;
//		std::cout << "initial_weight: " << initial_weight << std::endl;
		if (((weight_difference>0)&&(check_average_weight_in_buffer()
				- initial_weight) > weight_difference)|| ((weight_difference<0)
				&&(check_average_weight_in_buffer() - initial_weight)
						< weight_difference))

		{
//			std::cout << "w" << std::endl;
			// wszytkie potweridzenia warunku koncowego musza wystapic pod rzad
			if (!terminate_state_recognized)
			{
//				std::cout << "ponowna inicjalizacja" << std::endl;
				catch_lag = initial_catch_lag;
			}

			terminate_state_recognized = true;
			//    	printf("check_average_weight_in_buffer: %f, %f\n", check_average_weight_in_buffer(), initial_weight );
//			std::cout << "catch lag: " << catch_lag << std::endl;
			if ((--catch_lag) <= 0)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		else
		{
//			std::cout << "koniec" << std::endl;
			terminate_state_recognized = false;
			//return true;
			return false;
		}
	}

	return true;
}
