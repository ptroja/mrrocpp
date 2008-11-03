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
	fmg = new force_meassure_generator(*this,100000,20);	
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
	int option = choose_option ("1 - NoseGenerator, 2 - Test, 3 - Nacisk", 3);
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
		trajectories_test(10);
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
  
	ecp_termination_notice();
	ecp_wait_for_stop();
	cout<<"end\n"<<endl;
};

void ecp_task_tzu_test::naciskanie_test()
{
	tcg->set_tool_parameters(0,0,0); 
	tcg->Move();
	sleep(2);
	befg->Move();
	sr_ecp_msg->message("START NACISKANIE TEST");
	while(true)			
	{
		usleep(10);
		fmg->Move();
		cout<<"pomiar sily: "<<fmg->get_meassurement()<<endl;
	}
}

void ecp_task_tzu_test::nose_generator_test(int tool)
{
	while(true)			
	{
		sr_ecp_msg->message("START NOSE");
		
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
		sleep(2);
		befg->Move();
		tcg->set_tool_parameters(0,0,0); 
		tcg->Move();
		ynrfg->set_force_meassure(true);
		ynrfg->Move();
		break;
	}
}

// argumenty: ilosc powtorzen eksperymentu, taka sama dla parametrow wyliczonych i tych z common.ini
void ecp_task_tzu_test::trajectories_test(int count)
{
	Ft_v_vector result_common[count][10];
	Ft_v_vector result_wyliczone[count][10];
	// trajektorie dla ruchu
	double tmp[10][7] = {{0, -1.570795, 0, 0, 1.5707, 0, 0.07},
							   {0, -1.570795, 0, 1.5, 1.5707, 0, 0.07},
							   {0, -1.570795, 0, 0, 1.5707, 0, 0.07},
							   {0, -1.570795, 0, 0, 3, 0, 0.07},
							   {0, -1.570795, 0, 0, 0.1, 0, 0.07},
							   {0, -1.570795, 0, 0, 1.5707, 1.5707, 0.07},
							   {0, -1.570795, 0, 1.5, 1.5707, 1.5707, 0.07},
							   {0, -1.570795, 0, 0, 1.5707, 1.5707, 0.07},
							   {0, -1.570795, 0, 0, 3, 1.5707, 0.07},
							   {0, -1.570795, 0, 0, 0.1, 1.5707, 0.07}};	
	
	tcg->set_tool_parameters(0,0,0);
	tcg->Move();
	
	cout<<"START TRAJECTORIES TEST"<<endl;
	for(int j = 0 ; j < count ; j++)
	{	
		cout<<"parametry z common.ini: "<<j<<endl;
		//0.004 0.0 0.13		
		ftcg->set_tool_parameters(0.004,0.0,0.13,13.18);
		ftcg->Move();
		
		for(int i = 0 ; i < 10 ; i++)
		{
			sg->load_file_with_path(get_trajectory(tmp[i]));
			sg->Move();
			
			if(i == 0 || i == 5)
			{
				sleep(2);
				befg->Move();
			}
			
			fmg->Move();
			result_common[j][i] = fmg->get_meassurement();
			cout<<"pomiar "<<i<<": "<<fmg->get_meassurement()<<endl;
			str<<"pomiar "<<i<<": "<<fmg->get_meassurement()<<endl;
		}
	}
	
	for(int j = 0 ; j < count ; j++)
	{	
		cout<<"parametry wyliczone: "<<j<<endl;
		// weight: 13.4494
		// P_x: -0.00123971
		// P_y: -0.000875034
		// P_z: 0.136523

		ftcg->set_tool_parameters(-0.00123971, -0.000875034, 0.136523, 13.4494);
		ftcg->Move();
		
		for(int i = 0 ; i < 10 ; i++)
		{
			sg->load_file_with_path(get_trajectory(tmp[i]));
			sg->Move();
			
			if(i == 0 || i == 5)
			{
				sleep(2);
				befg->Move();
			}
			
			fmg->Move();
			result_wyliczone[j][i] = fmg->get_meassurement();
			cout<<"pomiar "<<i<<": "<<fmg->get_meassurement()<<endl;
			str<<"pomiar "<<i<<": "<<fmg->get_meassurement()<<endl;
		}
	}
	// result_common mamy wyniki przy ustawieniach z common.ini natomiast w result_wyliczone mamy wyniki otrzymane przy wyliczonch ustawieniach
	// roznica pomiedzy kolejnymi probami dla common.ini a tymi wyliczonymi
	//Ft_v_vector result_difference_common[count][10];
	//Ft_v_vector result_difference_wyliczone[count][10];
	
	for(int j = 0 ; j < count ; j++)
	{
		for(int i = 0 ; i < 10 ; i++)
		{
			for(int k = 0 ; k < 6 ; k++)
			{
				if(result_common[j][i][k] < 0)
					result_common[j][i][k] = result_common[j][i][k]*-1;
				if(result_wyliczone[j][i][k] < 0)
					result_wyliczone[j][i][k] = result_wyliczone[j][i][k]*-1;	
			}
			// result_difference_common[j][i] = result_common[j][i];
			// result_difference_wyliczone[j][i] = result_wyliczone[j][i];
		}
	}
	for(int j = 0 ; j < count ; j++)
	{	
		for(int i = 0 ; i < 10 ; i++)
		{
			cout<<"result_common: "<<endl<<result_common[j][i]<<endl;
			str<<"result_common: "<<endl<<result_common[j][i]<<endl;
		}
	}
	
	for(int j = 0 ; j < count ; j++)
	{	
		for(int i = 0 ; i < 10 ; i++)
		{
			cout<<"result_wyliczone: "<<endl<<result_wyliczone[j][i]<<endl;
			str<<"result_wyliczone: "<<endl<<result_wyliczone[j][i]<<endl;
		}
	}
//	cout<<"result_common: "<<endl<<result_common<<endl;
//	str<<"result_common: "<<endl<<result_common<<endl;
//	cout<<"result_wyliczone: "<<endl<<result_wyliczone<<endl;
//	str<<"result_wyliczone: "<<endl<<result_wyliczone<<endl;
	
	// do matlabowych wykresow
	
	ofstream matlab;
	matlab.open("../matlab.m"/*,ios::app*/);
	matlab<<"clear;"<<endl<<"x = 1:1:6;"<<endl<<"x1 = 0:.01:7;"<<endl;
	for(int j = 0 ; j < count ; j++)
	{
		for(int i = 0 ; i < 10 ; i++)
		{
			for(int k = 0 ; k < 6 ; k++)
			{
				matlab<<"yc_"<<j<<"_"<<i<<"("<<k<<")="<<result_common[j][i][k]<<";"<<endl;
			}
		}
	}
	
	for(int j = 0 ; j < count ; j++)
	{
		for(int i = 0 ; i < 10 ; i++)
		{
			for(int k = 0 ; k < 6 ; k++)
			{	
				matlab<<"yw_"<<j<<"_"<<i<<"("<<k<<")="<<result_wyliczone[j][i][k]<<";"<<endl;
			}
		}
	}
	
	matlab<<"figure(1);"<<endl;
	for(int d = 1 ; d < 6 ; d++)
	{
		matlab<<"subplot(3,2,"<<d<<");plot(";
		for(int j = 0 ; j < count ; j++)
		{
			matlab<<"x,yc_"<<j<<"_"<<d-1<<",'bx',";
		}
		
		for(int j = 0 ; j < count ; j++)
		{
			matlab<<"x,yw_"<<j<<"_"<<d-1<<",'rx',";
		}
				
		matlab<<"x1,0,'-g'"<<endl;	
	}	
	
	matlab<<"figure(2);"<<endl;
	for(int d = 1 ; d < 6 ; d++)
	{
		matlab<<"subplot(3,2,"<<d<<");plot(";
		for(int j = 0 ; j < count ; j++)
		{
			matlab<<"x,yc_"<<j<<"_"<<d-1+5<<",'bx',";
		}
		
		for(int j = 0 ; j < count ; j++)
		{
			matlab<<"x,yw_"<<j<<"_"<<d-1+5<<",'rx',";
		}
				
		matlab<<"x1,0,'-g'"<<endl;	
	}	
	matlab.close();
	// do kwadratu
	Ft_v_vector result_common_square[count][10];
	Ft_v_vector result_wyliczone_square[count][10];
	
	for(int j = 0 ; j < count ; j++)
	{
		for(int i = 0 ; i < 10 ; i++)
		{
			for(int k = 0 ; k < 6 ; k++)
			{
				result_common_square[j][i][k] = result_common[j][i][k]*result_common[j][i][k];
				result_wyliczone_square[j][i][k] = result_wyliczone[j][i][k]*result_wyliczone[j][i][k];
			}
		}
	}
	
	for(int j = 0 ; j < count ; j++)
	{	
		for(int i = 0 ; i < 10 ; i++)
		{
			cout<<"result_common_square: "<<endl<<result_common_square[j][i]<<endl;
			str<<"result_common_square: "<<endl<<result_common_square[j][i]<<endl;
		}
	}
	
	for(int j = 0 ; j < count ; j++)
	{	
		for(int i = 0 ; i < 10 ; i++)
		{
			cout<<"result_wyliczone_square: "<<endl<<result_wyliczone_square[j][i]<<endl;
			str<<"result_wyliczone_square: "<<endl<<result_wyliczone_square[j][i]<<endl;
		}
	}
//	cout<<"result_common_square: "<<endl<<result_common_square<<endl;
//	str<<"result_common_square: "<<endl<<result_common_square<<endl;
//	cout<<"result_wyliczone_square: "<<endl<<result_wyliczone_square<<endl;
//	str<<"result_wyliczone_square: "<<endl<<result_wyliczone_square<<endl;

	// do matlabowych wykresow
//	for(int j = 0 ; j < count ; j++)
//	{
//		for(int i = 0 ; i < 10 ; i++)
//		{
//			for(int k = 0 ; k < 6 ; k++)
//			{
//				str<<k<<" "<<result_difference_common_square[j][i][k]<<endl;
//			}
//		}
//	}
//	
//	for(int j = 0 ; j < count ; j++)
//	{
//		for(int i = 0 ; i < 10 ; i++)
//		{
//			for(int k = 0 ; k < 6 ; k++)
//			{
//				str<<k<<" "<<result_difference_wyliczone_square[j][i][k]<<endl;
//			}
//		}
//	}
	
	// suma bledow
	double sum_of_errors_common[count];
	double sum_of_errors_wyliczone[count];
	for(int i = 0 ; i < count ; i++)
	{
		sum_of_errors_common[i] = 0;
		sum_of_errors_wyliczone[i] = 0;
	} 
	
	for(int j = 0 ; j < count ; j++)
	{
		for(int i = 0 ; i < 10 ; i++)
		{
			for(int k = 0 ; k < 6 ; k++)
			{
				sum_of_errors_common[j] += result_common[j][i][k];
				sum_of_errors_wyliczone[j] += result_wyliczone[j][i][k];
			}
		}
	}
	
	double sum_common = 0;
	for(int j = 0 ; j < count ; j++)
	{
		cout<<"sum_of_errors_common: "<<sum_of_errors_common[j]<<endl;
		str<<"sum_of_errors_common: "<<sum_of_errors_common[j]<<endl;
		sum_common += sum_of_errors_common[j];
	}
	cout<<"sum_of_errors_common_general: "<<sum_common<<endl;
	str<<"sum_of_errors_common_general: "<<sum_common<<endl;
	
	double sum_wyliczone = 0;
	for(int j = 0 ; j < count ; j++)
	{
		cout<<"sum_of_errors_wyliczone: "<<sum_of_errors_wyliczone[j]<<endl;
		str<<"sum_of_errors_wyliczone: "<<sum_of_errors_wyliczone[j]<<endl;
		sum_wyliczone += sum_of_errors_wyliczone[j];
	}
	cout<<"sum_of_errors_wyliczone_general: "<<sum_wyliczone<<endl;
	str<<"sum_of_errors_wyliczone_general: "<<sum_wyliczone<<endl;
	
//	cout<<"sum_of_errors_common: "<<sum_of_errors_common<<endl;
//	str<<"sum_of_errors_common: "<<sum_of_errors_common<<endl;
//	cout<<"sum_of_errors_wyliczone: "<<sum_of_errors_wyliczone<<endl;
//	str<<"sum_of_errors_wyliczone: "<<sum_of_errors_wyliczone<<endl;
	// pomyslec jeszcze jakie porownania mozna zrobic
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


const char* ecp_task_tzu_test::get_trajectory(double x[])
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
	init_meassurement_count = _meassurement_count;
}

/** ustawienie konfiguracji generatora **/
bool force_meassure_generator::set_configuration(int _sleep_time, int _meassurement_count)
{
	sleep_time = _sleep_time; 
	meassurement_count = _meassurement_count;
	init_meassurement_count = _meassurement_count;
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

/** next step **/
bool force_meassure_generator::next_step()
{
	usleep(sleep_time);
	Homog_matrix current_frame_wo_offset(the_robot->EDP_data.current_arm_frame);
	current_frame_wo_offset.remove_translation();
	
	Ft_v_vector force_torque(the_robot->EDP_data.current_force_xyz_torque_xyz);
	weight += force_torque;

//	cout<<"force_torque: "<<force_torque<<endl;
	meassurement_count--;
	if(meassurement_count <= 0)
	{
		meassurement_count = init_meassurement_count;
		for(int i = 0 ; i < 6 ; i++)
			weight[i] = weight[i]/meassurement_count;
		return false;
	}
	return true;
}

Ft_v_vector& force_meassure_generator::get_meassurement()
{
//	for(int i = 0 ; i < 6 ; i++)
//		weight[i] = weight[i]/meassurement_count;
	return weight;
}
