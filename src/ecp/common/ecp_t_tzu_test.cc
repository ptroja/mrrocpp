#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include <fstream>

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
	str.open("../results_test.txt",ios::app);
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
	
	
	int option = choose_option ("1 - NoseGenerator, 2 - Test", 2);
	if (option == OPTION_ONE)
    {
    		sr_ecp_msg->message("NoseGenerator");
   		procedure_type = NOSE;
   	}
    else if (option == OPTION_TWO)
    {
 		sr_ecp_msg->message("Test");
		procedure_type = TEST;
	}
  
	set_trajectories();
    if(procedure_type == NOSE)
    {
		nose_generator_test();
	}
	else if(procedure_type == TEST)
	{
		trajectories_test();
	}
	ecp_termination_notice();
	ecp_wait_for_stop();
	std::cout<<"end\n"<<std::endl;
};

void ecp_task_tzu_postument_test::nose_generator_test(void)
{
	while(true)			
	{
		cout<<"START NOSE"<<endl;
		befg->Move();
		cout<<"Biasowanie dokonane"<<endl;
		tcg->set_tool_parameters(0,0,0.09); 
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
	while(true)			
	{
		cout<<"START TRAJECTORIES TEST"<<endl;
		befg->Move();
		cout<<"Biasowanie dokonane"<<endl;
		tcg->set_tool_parameters(0,0,0.09); // to tutaj chyba trzeba przesunac o te 25 cm czyli argumenty powinny wygladac jakos tak (0,0,0.25)
		tcg->Move();
		cout<<"Puszczenie nose generatora"<<endl;
		sleep(1);

		break;
	}
}

void ecp_task_tzu_postument_test::set_trajectories() // mozna wywalic zmienna robot z klasy i wtedy jawnie przekazywac ja tu do funkcji
{
	if(robot == ON_TRACK)
	{
		test_trajectories[0] = "../trj/tzu/test/on_track/tzu_1_on_track.trj";
		test_trajectories[1] = "../trj/tzu/test/on_track/tzu_2_on_track.trj";
		test_trajectories[2] = "../trj/tzu/test/on_track/tzu_3_on_track.trj";
		test_trajectories[3] = "../trj/tzu/test/on_track/tzu_4_on_track.trj";
		test_trajectories[4] = "../trj/tzu/test/on_track/tzu_5_on_track.trj";
		test_trajectories[5] = "../trj/tzu/test/on_track/tzu_6_on_track.trj";
		test_trajectories[6] = "../trj/tzu/test/on_track/tzu_7_on_track.trj";
		test_trajectories[7] = "../trj/tzu/test/on_track/tzu_8_on_track.trj";
	}	
	else if(robot == POSTUMENT)
	{
		test_trajectories[0] = "../trj/tzu/test/postument/tzu_1_on_track.trj";
		test_trajectories[1] = "../trj/tzu/test/postument/tzu_2_on_track.trj";
		test_trajectories[2] = "../trj/tzu/test/postument/tzu_3_on_track.trj";
		test_trajectories[3] = "../trj/tzu/test/postument/tzu_4_on_track.trj";
		test_trajectories[4] = "../trj/tzu/test/postument/tzu_5_on_track.trj";
		test_trajectories[5] = "../trj/tzu/test/postument/tzu_6_on_track.trj";
		test_trajectories[6] = "../trj/tzu/test/postument/tzu_7_on_track.trj";
		test_trajectories[7] = "../trj/tzu/test/postument/tzu_8_on_track.trj";
	}
}
// reszte skonczyc w domu, bo przeciez do opracowania tego nie bede potrzebowal robota
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
		cout<<"pomiar: "<<weight<<endl; 
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
#include <iostream>
