#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "ecp_mp/ecp_mp_s_force.h"
#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"	

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_t_tzu_test.h"
#include "lib/mathtr.h"
#include "ecp/common/ecp_g_smooth.h"

using namespace std;
/** konstruktor **/
ecp_task_tzu_postument_test::ecp_task_tzu_postument_test(configurator &_config) : ecp_task(_config)
{
	befg = NULL;
	ftcg = NULL;
	tcg = NULL;
	fmg = NULL;
	ynrfg = NULL;
	sg = NULL;
	str.open("../results_test.txt"/*,ios::app*/);
};

/** destruktor **/
ecp_task_tzu_postument_test::~ecp_task_tzu_postument_test()
{
	str<<"--- KONIEC ---"<<endl;
	str.close();
};


// methods for ECP template to redefine in concrete classes
void ecp_task_tzu_postument_test::task_initialization(void) 
{
	// ecp_m_robot = new ecp_irp6_on_track_robot (*this);
	if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
	{
		ecp_m_robot = new ecp_irp6_on_track_robot (*this);
		robot = ON_TRACK;
	}
	else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
	{
		ecp_m_robot = new ecp_irp6_postument_robot (*this);
		robot = POSTUMENT;
	}
	
	// inicjalizacja generatorow
	sg = new ecp_smooth_generator (*this, true, false);
	befg = new bias_edp_force_generator(*this);
	fmg = new force_meassure_generator(*this);	
	ftcg = new ecp_force_tool_change_generator(*this);
	tcg = new ecp_tool_change_generator(*this,true);
	ynrfg = new ecp_tff_nose_run_generator(*this,8);
	
	sr_ecp_msg->message("ECP loaded");
};

void ecp_task_tzu_postument_test::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP cs irp6ot  - pushj start in tzu");
	ecp_wait_for_start();
	
	
	int option = choose_option ("1 - NoseGenerator, 2 - Test, 3 - Eksperyment", 3);
	int tool = -1;
	if (option == OPTION_ONE)
    {
    		sr_ecp_msg->message("NoseGenerator");
   		procedure_type = NOSE;
 		option = choose_option ("1 - std, 2 - wyliczony", 2);
	  	if (option == OPTION_ONE)
    			tool = 0;
		else if (option == OPTION_TWO)
    			tool = 1;
	}
    else if (option == OPTION_TWO)
    {
 		sr_ecp_msg->message("Test");
		procedure_type = TEST;
	}
  	else if (option == OPTION_THREE)
    {
 		sr_ecp_msg->message("Eksperyment z naciskaniem");
		procedure_type = EKSPERYMENT;
	}
  
	set_trajectories();
    if(procedure_type == NOSE)
    {
		nose_generator_test(tool);
	}
	else if(procedure_type == TEST)
	{
		trajectories_test();
	}
	else if(procedure_type == EKSPERYMENT)
	{
		naciskanie_test();
	}
	ecp_termination_notice();
	ecp_wait_for_stop();
	std::cout<<"end\n"<<std::endl;
};

void ecp_task_tzu_postument_test::naciskanie_test()
{
	tcg->set_tool_parameters(0,0,0); 
	tcg->Move();
//	sg->load_file_with_path(get_trajectorie(0.759997, -1.5707, 0, 1.5707, 1.5707, 0 ,0.07 ,POSTUMENT));
//	sg->Move();
	befg->Move();
	cout<<"START NACISKANIE TEST"<<endl;
	while(true)			
	{
		usleep(10);
		fmg->Move();
		cout<<"pomiar: "<<fmg->weight<<endl;
	}
}

void ecp_task_tzu_postument_test::nose_generator_test(int tool)
{
	while(true)			
	{
		cout<<"START NOSE"<<endl;
		char* trj;
		trj = "../trj/tzu/standard/postument/tzu_2_postument.trj";
		if(robot == ON_TRACK)
			trj = "../trj/tzu/standard/on_track/tzu_2_on_track.trj";
		
		if(tool == 0)
		{
			cout<<"parametry z common.ini"<<endl;
			ftcg->set_tool_parameters(0.004,0.0,0.156,10.8);
			ftcg->Move();
		}
		else if(tool == 1)
		{
			cout<<"parametry wyliczone"<<endl;
			ftcg->set_tool_parameters(-0.00622889, -0.000365208, 0.167449, 13.2519);
			ftcg->Move();
		}
		
		sg->load_file_with_path(trj);
		sg->Move();
		befg->Move();
		cout<<"Biasowanie dokonane"<<endl;
		tcg->set_tool_parameters(0,0,0); 
		tcg->Move();
		cout<<"Puszczenie nose generatora"<<endl;
		sleep(1);
		ynrfg->set_force_meassure(true);
		ynrfg->Move();
		break;
	}
}

void ecp_task_tzu_postument_test::trajectories_test(void)
{
/*
	while(true)			
	{
		cout<<"START TRAJECTORIES TEST"<<endl;
		// befg->Move();
		cout<<"Biasowanie dokonane"<<endl;
		//tcg->set_tool_parameters(0,0,0.25); // to tutaj chyba trzeba przesunac o te 25 cm czyli argumenty powinny wygladac jakos tak (0,0,0.25)
		//tcg->Move();
		cout<<"Rozpoczecie testowania dla roznych trajektorii"<<endl;
		
		for(int i = 0 ; i < NUMBER_OF_TEST_TRAJECTORIES ; i++)
		{
			if(i == 0)	// zbiasowanie czujnika dla pierwszego polozenia, w takim wypadku odczyt sily jaki op tym nastapi pewnie nie bedzie mial wiekszego sensu
			{	
				cout<<"pomiar przed biasem "<<i<<": "<<fmg->weight<<endl;
				str<<"pomiar przed biasem "<<i<<": "<<fmg->weight<<endl;
				befg->Move();
				fmg->Move();
				cout<<"pomiar po biasie "<<i<<": "<<fmg->weight<<endl;
				str<<"pomiar po biasie "<<i<<": "<<fmg->weight<<endl;
			}
			sg->load_file_with_path(test_trajectories[i]);
			sg->Move();
			fmg->Move();
			cout<<"pomiar "<<i<<": "<<fmg->weight<<endl;
			str<<"pomiar "<<i<<": "<<fmg->weight<<endl;
		}
		// pomiar w pozycji wyjsciowej
		sg->load_file_with_path(test_trajectories[0]);
		sg->Move();
		fmg->Move();
		cout<<"pomiar "<<0<<": "<<fmg->weight<<endl;
		str<<"pomiar "<<0<<": "<<fmg->weight<<endl;
		sleep(1);

		break;
	}
*/	

	tcg->set_tool_parameters(0,0,0); // to tutaj chyba trzeba przesunac o te 25 cm czyli argumenty powinny wygladac jakos tak (0,0,0.25)
	tcg->Move();
	for(int j = 0 ; j < 4 ; j++)
	{
		cout<<"START TRAJECTORIES TEST"<<endl;
		// parametry modelu z common.ini
//		cout<<"parametry z common.ini"<<endl;
//		ftcg->set_tool_parameters(0.004,0.0,0.156,10.8);
//		ftcg->Move();
		// parametry modelu z procedury wyznaczania modelu
//		cout<<"parametry wyliczone"<<endl;
//		ftcg->set_tool_parameters(-0.00311298, -0.00382255, -0.1010115, 13.179675);
//		ftcg->Move();
		if(j == 0 || j == 2)
		{
			test_trajectories[0] = "../trj/tzu/0_o.trj";
			test_trajectories[1] = "../trj/tzu/1_o.trj";
			test_trajectories[2] = "../trj/tzu/0_o.trj";
			test_trajectories[3] = "../trj/tzu/2_o.trj";
			test_trajectories[4] = "../trj/tzu/3_o.trj";
		}
		if(j == 1 || j == 3)
		{
			test_trajectories[0] = "../trj/tzu/0.trj";
			test_trajectories[1] = "../trj/tzu/1.trj";
			test_trajectories[2] = "../trj/tzu/0.trj";
			test_trajectories[3] = "../trj/tzu/2.trj";
			test_trajectories[4] = "../trj/tzu/3.trj";
		}
		
		if(j == 0 || j == 1)
		{
			cout<<"parametry z common.ini"<<endl;
			ftcg->set_tool_parameters(0.004,0.0,0.156,10.8);
			ftcg->Move();
		}
		else if(j == 2 || j == 3)
		{
			cout<<"parametry wyliczone"<<endl;
			ftcg->set_tool_parameters(-0.00622889, -0.000365208, 0.167449, 13.2519);
			ftcg->Move();
		}
		
		for(int i = 0 ; i < 5 ; i++)
		{
			sg->load_file_with_path(test_trajectories[i]);
			sg->Move();
			if(i == 0)
				befg->Move();
			fmg->Move();
			cout<<"pomiar "<<i<<": "<<fmg->weight<<endl;
			str<<"pomiar "<<i<<": "<<fmg->weight<<endl;
		}
	}
}

void ecp_task_tzu_postument_test::set_trajectories() // mozna wywalic zmienna robot z klasy i wtedy jawnie przekazywac ja tu do funkcji
{
	// sprawdzic czy wszystkie tak wrzucone ruchy maja wiekszy sens
	if(robot == ON_TRACK)
	{
		test_trajectories[0] = "../trj/tzu/standard/on_track/tzu_3_on_track.trj";
		test_trajectories[1] = "../trj/tzu/standard/on_track/tzu_2_on_track.trj";
		test_trajectories[2] = "../trj/tzu/standard/on_track/tzu_1_on_track.trj";
		test_trajectories[3] = "../trj/tzu/alternative/on_track/x_weight_meassure/method_1/tzu_1_on_track.trj";
		test_trajectories[4] = "../trj/tzu/alternative/on_track/x_weight_meassure/method_1/tzu_2_on_track.trj";
		test_trajectories[5] = "../trj/tzu/alternative/on_track/x_weight_meassure/method_2/tzu_1_on_track.trj";
		test_trajectories[6] = "../trj/tzu/alternative/on_track/x_weight_meassure/method_2/tzu_2_on_track.trj";
		test_trajectories[7] = "../trj/tzu/alternative/on_track/y_weight_meassure/method_1/tzu_1_on_track.trj";
		test_trajectories[8] = "../trj/tzu/alternative/on_track/y_weight_meassure/method_1/tzu_2_on_track.trj";
		test_trajectories[9] = "../trj/tzu/alternative/on_track/y_weight_meassure/method_2/tzu_1_on_track.trj";
		test_trajectories[10] = "../trj/tzu/alternative/on_track/y_weight_meassure/method_2/tzu_2_on_track.trj";
	}	
	else if(robot == POSTUMENT)
	{

		test_trajectories[0] = "../trj/tzu/standard/postument/tzu_3_postument.trj";
		test_trajectories[1] = "../trj/tzu/standard/postument/tzu_2_postument.trj";
		test_trajectories[2] = "../trj/tzu/standard/postument/tzu_1_postument.trj";
		test_trajectories[3] = "../trj/tzu/alternative/postument/x_weight_meassure/method_1/tzu_1_postument.trj";
		test_trajectories[4] = "../trj/tzu/alternative/postument/x_weight_meassure/method_1/tzu_2_postument.trj";
		test_trajectories[5] = "../trj/tzu/alternative/postument/x_weight_meassure/method_2/tzu_1_postument.trj";
		test_trajectories[6] = "../trj/tzu/alternative/postument/x_weight_meassure/method_2/tzu_2_postument.trj";
		test_trajectories[7] = "../trj/tzu/alternative/postument/y_weight_meassure/method_1/tzu_1_postument.trj";
		test_trajectories[8] = "../trj/tzu/alternative/postument/y_weight_meassure/method_1/tzu_2_postument.trj";
		test_trajectories[9] = "../trj/tzu/alternative/postument/y_weight_meassure/method_2/tzu_1_postument.trj";
		test_trajectories[10] = "../trj/tzu/alternative/postument/y_weight_meassure/method_2/tzu_2_postument.trj";
/*
		test_trajectories[0] = "../trj/tzu/standard/postument/tzu_2_postument.trj";
		test_trajectories[1] = "../trj/tzu/standard/postument/tzu_3_postument.trj";
		test_trajectories[2] = "../trj/tzu/standard/postument/tzu_2_postument.trj";
		test_trajectories[3] = "../trj/tzu/alternative/postument/y_weight_meassure/method_2/tzu_1_postument.trj";
		test_trajectories[4] = "../trj/tzu/alternative/postument/y_weight_meassure/method_2/tzu_2_postument.trj";
*/
	}
}


char* ecp_task_tzu_postument_test::get_trajectorie(double x1, double x2, double x3, double x4, double x5, double x6 ,double x7 ,int robot) // mozna wywalic zmienna robot z klasy i wtedy jawnie przekazywac ja tu do funkcji
{
	ofstream temp;
	//char* file = "../trj/tzu/temp.trj";
	temp.open("../trj/tzu/temp.trj");
	temp<<"JOINT"<<endl;
	temp<<"1"<<endl;
	temp<<"0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0"<<endl;
	temp<<"0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0"<<endl;
	temp<<"0.3 0.3 0.3 0.3 0.3 0.3 0.3 1.0"<<endl;
	temp<<"0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.01"<<endl;
	if(robot == POSTUMENT)
		temp<<x1<<" "<<x2<<" "<<x3<<" "<<x4<<" "<<x5<<" "<<x6<<" "<<x7<<" 0"<<endl;
	else if(robot == ON_TRACK)
		temp<<"0 "<<x1<<" "<<x2<<" "<<x3<<" "<<x4<<" "<<x5<<" "<<x6<<" "<<x7<<endl;
	temp.close();
	return "../trj/tzu/temp.trj";
}
ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_tzu_postument_test(_config);
};

// zobaczyc czy ten generator bedzie tu potrzebny
/**** force meassure generator ****/

/** konstruktor **/
force_meassure_generator::force_meassure_generator(ecp_task& _ecp_task, int _sleep_time, int _meassurement_count) :
	ecp_generator(_ecp_task)
{
	sleep_time = _sleep_time; 
	meassurement_count = _meassurement_count;
}

/** ustawienie konfiguracji generatora **/
bool force_meassure_generator::set_configuration(int _sleep_time, int _meassurement_count)
{
	sleep_time = _sleep_time; 
	meassurement_count = _meassurement_count;
}

/** first step **/
bool force_meassure_generator::first_step()
{
	the_robot->EDP_data.instruction_type = GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.get_arm_type = FRAME;
	the_robot->EDP_data.next_interpolation_type
			= TCIM;
	for(int i = 0; i < 6 ; i++)
		weight[i] = 0;
	
	return true;
}

Ft_v_vector *force_meassure_generator::get_meassurement()
{
	return &weight;
}

/** next step **/
bool force_meassure_generator::next_step()
{
	//Ft_v_vector average_meassurement[meassurement_count];
	sleep(2);
	for(int i = 0 ; i < meassurement_count ; i++)
	{
//		cout<<"pomiar: "<<weight<<endl; 
		Homog_matrix current_frame_wo_offset(the_robot->EDP_data.current_arm_frame);
		current_frame_wo_offset.remove_translation();
	
		Ft_v_vector force_torque(the_robot->EDP_data.current_force_xyz_torque_xyz);
		weight += force_torque;
		weight = force_torque;
		sleep(sleep_time);
	}
	for(int i = 0 ; i < 6 ; i++)
		weight[i] = weight[i]/meassurement_count;
	return false;
}
