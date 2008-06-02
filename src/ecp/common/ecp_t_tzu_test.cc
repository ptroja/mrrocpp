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
ecp_task_tzu_test::ecp_task_tzu_test(configurator &_config) : ecp_task(_config)
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
ecp_task_tzu_test::~ecp_task_tzu_test()
{
	str<<"--- KONIEC ---"<<endl;
	str.close();
};

void ecp_task_tzu_test::force_measurrement()
{
	sleep(1);
	double pom_tmp[6];
	for(int i = 0 ; i < 6 ; i++)
		pom_tmp[i] = 0;
		
	for(int i = 0 ; i < 20 ; i++)
	{
		fmg->Move();
		// cout<<"pomiar1: "<<fmg->weight<<endl;
		for(int j = 0 ; j < 6 ; j++)
			pom_tmp[j] += fmg->weight[j];
	}
	
	// cout<<"pomiar2: "<<pom_tmp<<endl;	
	
	for(int i = 0 ; i < 6 ; i++)
		pom[i] = pom_tmp[i]/20;
	
//	for(int i = 0 ; i < 6 ; i++)
//	 cout<<"pomiar3: "<<pom[i]<<endl;
};

// methods for ECP template to redefine in concrete classes
void ecp_task_tzu_test::task_initialization(void) 
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
	fmg = new force_meassure_generator(*this,0,1);	
	ftcg = new ecp_force_tool_change_generator(*this);
	tcg = new ecp_tool_change_generator(*this,true);
	ynrfg = new ecp_tff_nose_run_generator(*this,8);
	
	sr_ecp_msg->message("ECP loaded");
};

void ecp_task_tzu_test::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP cs irp6ot  - pushj start in tzu");
	ecp_wait_for_start();
	
	set_trajectories();
	int option = choose_option ("1 - NoseGenerator, 2 - Test, 3 - Nacisk, 4 - Eksperyment", 4);
	if (option == OPTION_ONE)
    {
    		/*
     	 * reczne obroty manipulatora przy ustawionym standardowym modelu (dane z common.ini) oraz przy modelu wyznaczonym przy 
     	 * pomocy stworzonej automatycznej procedury do wyznaczania narzedzia
     	 */  		
		sr_ecp_msg->message("NoseGenerator");
 		option = choose_option ("1 - std, 2 - wyliczony", 2);
	  	if (option == OPTION_ONE)
    			nose_generator_test(0);
		else if (option == OPTION_TWO)
    			nose_generator_test(1);
	}
    else if (option == OPTION_TWO)
    {
    		/*
		 * test polegajacy na obrotach manipulatora do kilku roznych pozycji z dwiema roznymi orientacjami chwytaka dla modelu narzedzia
		 * pobranego z common.ini oraz dla modelu wyznaczonego poprzez stworzona do tego procedure oraz obserwacji zmian odczytow
		 * sil dla tych roznych przypadkow
		 */
 		sr_ecp_msg->message("Test");
		trajectories_test();
	}
  	else if (option == OPTION_THREE)
    {
 		/*
		 * test polegajacy na sprawdzeniu zmiany odczytow sil dla nacisku wywieranego tylko na jedna z osi np. na os x i sprawdzaniu czy
		 * odkladaja sie jakies sily na pozostalych osiach, chociaz nie powinny
		 */
		sr_ecp_msg->message("Nacisk");
		naciskanie_test();
	}
	else if (option == OPTION_FOUR)
    {
 		/*
		 * test modelu ... robota
		 */
		sr_ecp_msg->message("Eksperyment");
		eksperyment_test();
	}
  
	ecp_termination_notice();
	ecp_wait_for_stop();
	cout<<"end\n"<<endl;
};


void ecp_task_tzu_test::eksperyment_test()
{
	sr_ecp_msg->message("START EKSPERYMENT TEST");
	

}
void ecp_task_tzu_test::naciskanie_test()
{
	tcg->set_tool_parameters(0,0,0); 
	tcg->Move();
//   aktualnie manipulator ustawiany jest recznie do odpowiedniej pozycji, a nastepnie wykonywane sa pomiary
//	sg->load_file_with_path(get_trajectorie(0.759997, -1.5707, 0, 1.5707, 1.5707, 0 ,0.07 ,POSTUMENT));
//	sg->Move();
	befg->Move();
	sr_ecp_msg->message("START NACISKANIE TEST");
	while(true)			
	{
		usleep(10);
		// fmg->Move();
		force_measurrement();
		cout<<"pomiar: ";
		for(int i = 0 ; i < 6 ; i++)
			cout<<pom[i]<<", ";
		cout<<endl;
		// cout<<"pomiar: "<<fmg->weight<<endl;
	}
}

void ecp_task_tzu_test::nose_generator_test(int tool)
{
	while(true)			
	{
		sr_ecp_msg->message("START NOSE");
		
		// 0.004 0.0 0.13
		// weight=13.18
		if(tool == 0)
		{
			sr_ecp_msg->message("parametry z common.ini");
			ftcg->set_tool_parameters(0.004,0.0,0.13,13.18);
			ftcg->Move();
		}
		else if(tool == 1)
		{
			sr_ecp_msg->message("parametry wyliczone");
			ftcg->set_tool_parameters(-0.00622889, -0.000365208, 0.167449, 13.2519);
			ftcg->Move();
		}
		
		// ustawienie manipulatora pionowo do gory
		double tmp[] = {0.759997, -1.5707, 0, 1.132, 1.5707, 0 ,0.07};
		sg->load_file_with_path(get_trajectory(tmp));
		sg->Move();
		befg->Move();
		sr_ecp_msg->message("biasowanie dokonane");
		tcg->set_tool_parameters(0,0,0); 
		tcg->Move();
		ynrfg->set_force_meassure(true);
		ynrfg->Move();
		break;
	}
}

void ecp_task_tzu_test::trajectories_test(void)
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

	// trajektorie dla ruchu
	double tmp[2][5][7] = {{{0, -1.570795, 0, 0, 1.5707, 0, 0.07},
							       {0, -1.570795, 0, 1.5, 1.5707, 0, 0.07},
							       {0, -1.570795, 0, 0, 1.5707, 0, 0.07},
							       {0, -1.570795, 0, 0, 3, 0, 0.07},
							       {0, -1.570795, 0, 0, 0.1, 0, 0.07}},
								{{0, -1.570795, 0, 0, 1.5707, 1.5707, 0.07},
							       {0, -1.570795, 0, 1.5, 1.5707, 1.5707, 0.07},
							       {0, -1.570795, 0, 0, 1.5707, 1.5707, 0.07},
							       {0, -1.570795, 0, 0, 3, 1.5707, 0.07},
							       {0, -1.570795, 0, 0, 0.1, 1.5707, 0.07}}};	
	
	tcg->set_tool_parameters(0,0,0);
	tcg->Move();
	for(int j = 0 ; j < 4 ; j++)
	{	
		cout<<"START TRAJECTORIES TEST"<<endl;
		if(j == 0 || j == 1)
		{
			cout<<"parametry z common.ini"<<endl;
			ftcg->set_tool_parameters(0.004,0.0,0.156,10.8);
			ftcg->Move();
		}
		else if(j == 2 || j == 3)
		{
			cout<<"parametry wyliczone"<<endl;
			cout<<"weight: 13.1243, P_x: 0.007698, P_y: 0.000848, P_z: 0.161029"<<endl;
			// weight: 13.0715
			// P_x: -0.00562063
			// P_y: -0.000988634
			// P_z: 0.157906
			//ftcg->set_tool_parameters(-0.00562063, -0.000988634, 0.157906, 13.0715);
			// weight: 13.745
			// P_x: -0.00524009
			// P_y: -0.00149684
			// P_z: 0.150014
			// ftcg->set_tool_parameters(-0.00524009, -0.00149684, 0.150014, 13.745);
			// weight: 13.1884
			// P_x: -0.000333794
			// P_y: 0.000202744
			// P_z: 0.138832
			ftcg->set_tool_parameters(-0.000333794, 0.000202744, 0.138832, 13.1884);
			ftcg->Move();
		}
			
		for(int i = 0 ; i < 5 ; i++)
		{
			sg->load_file_with_path(get_trajectory(tmp[j%2][i]));
			sg->Move();
			if(i == 0)
			{
				sleep(2);
				befg->Move();
			}
			// fmg->Move();
			force_measurrement();
			// cout<<"pomiar "<<i<<": "<<fmg->weight<<endl;
			cout<<"pomiar: ";
			for(int k = 0 ; k < 6 ; k++)
				cout<<pom[k]<<", ";
			cout<<endl;
		
			str<<"pomiar "<<i<<": "<<fmg->weight<<endl;
		}
	}
}

void ecp_task_tzu_test::set_trajectories() // mozna wywalic zmienna robot z klasy i wtedy jawnie przekazywac ja tu do funkcji
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
	}
}


char* ecp_task_tzu_test::get_trajectory(double x[])
{
	ofstream temp;
	temp.open("../trj/tzu/temp.trj");
	temp<<"JOINT"<<endl;
	temp<<"1"<<endl;
	temp<<"0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0"<<endl;
	temp<<"0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0"<<endl;
	temp<<"0.3 0.3 0.3 0.3 0.3 0.3 0.3 1.0"<<endl;
	temp<<"0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.01"<<endl;
	if(robot == POSTUMENT)
		temp<<x[0]<<" "<<x[1]<<" "<<x[2]<<" "<<x[3]<<" "<<x[4]<<" "<<x[5]<<" "<<x[6]<<" 0"<<endl;
	else if(robot == ON_TRACK)
		temp<<"0 "<<x[0]<<" "<<x[1]<<" "<<x[2]<<" "<<x[3]<<" "<<x[4]<<" "<<x[5]<<" "<<x[6]<<endl;
	temp.close();
	return "../trj/tzu/temp.trj";
}

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_tzu_test(_config);
};


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
//	cout<<"sleep time: "<<sleep_time<<endl;
//	cout<<"meassurement_count: "<<meassurement_count<<endl;
	//sleep(sleep_time);
	for(int i = 0 ; i < meassurement_count ; i++)
	{
		Homog_matrix current_frame_wo_offset(the_robot->EDP_data.current_arm_frame);
		current_frame_wo_offset.remove_translation();
	
		Ft_v_vector force_torque(the_robot->EDP_data.current_force_xyz_torque_xyz);
		weight += force_torque;
		weight = force_torque;
		usleep(50000);
//		cout<<"force torque: "<<force_torque<<endl;
	}
	
//	cout<<"weight przed: "<<weight<<endl;
	for(int i = 0 ; i < 6 ; i++)
		weight[i] = weight[i]/meassurement_count;
//	cout<<"weight po: "<<weight<<endl;
	return false;
}
