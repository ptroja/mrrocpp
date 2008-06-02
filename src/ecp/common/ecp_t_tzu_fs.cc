#include <iostream>
#include <fstream>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp_mp/ecp_mp_s_force.h"
#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"	

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_t_tzu_fs.h"
#include "lib/mathtr.h"
#include "ecp/common/ecp_g_smooth.h"

/*
 * przy najblizszej okazji zrobic tu porzadek przy wykorzystaniu jakiegos porzadnego edytora!!!
 */
using namespace std;
/** konstruktor **/
ecp_task_tzu_fs::ecp_task_tzu_fs(configurator &_config) : ecp_task(_config)
{
	sg = NULL;
	befg = NULL;
	ftcg = NULL;
	tcg = NULL;
	etnrg = NULL;
};

/** destruktor **/
ecp_task_tzu_fs::~ecp_task_tzu_fs()
{
	str<<"--- KONIEC ---"<<endl;
	str.close();
};

void ecp_task_tzu_fs::force_measurrement()
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
void ecp_task_tzu_fs::task_initialization(void) 
{
	if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
	{
		ecp_m_robot = new ecp_irp6_on_track_robot (*this);
		robot = ON_TRACK;
		str.open("../on_track_results.txt");
	}
	else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
	{
		ecp_m_robot = new ecp_irp6_postument_robot (*this);
		robot = POSTUMENT;
		str.open("../postument_results.txt");
	}
	
	sg = new ecp_smooth_generator (*this, true, false);
	befg = new bias_edp_force_generator(*this);
	fmg = new force_meassure_generator(*this,0,1);	
	ftcg = new ecp_force_tool_change_generator(*this);
	tcg = new ecp_tool_change_generator(*this,true);
	etnrg = new ecp_tff_nose_run_generator(*this,8);
	sr_ecp_msg->message("ECP loaded");
};

void ecp_task_tzu_fs::main_task_algorithm(void)
{
	bool automatic = false;
	bool correction = false;
	sr_ecp_msg->message("ECP cs irp6ot  - pushj start in tzu");
//	str.open("../results.txt");
//	str<<"test"<<endl;
//	str.close();
	ecp_wait_for_start();
	int procedure_type;
	char* additional_move;
	
	if(robot == ON_TRACK)
		additional_move = "../trj/tzu/tzu_on_track_1.trj";
	else
		additional_move = "../trj/tzu/tzu_postument_1.trj";

	int option = choose_option ("1 - Standard, 2 - Alternative, 3 - Auto, 4 - Correction", 4);
	if (option == OPTION_ONE)
    {
    		sr_ecp_msg->message("Wyznaczanie modelu metoda standardowa");
   		procedure_type = STANDARD;
   	}
    else if (option == OPTION_TWO)
    {
		sr_ecp_msg->message("Wyznaczanie modelu metoda alternatywna x");
		option = choose_option ("1 - x1, 2 - x2, 3 - y1, 4 - y2", 4);
		if (option == OPTION_ONE)
		{
			sr_ecp_msg->message("Wyznaczanie modelu metoda alternatywna x1");
			procedure_type = ALTERNATIVE_X_METHOD_1;
		}
		else if (option == OPTION_TWO)
		{
			sr_ecp_msg->message("Wyznaczanie modelu metoda alternatywna x2");
			procedure_type = ALTERNATIVE_X_METHOD_2;
		}
		else if (option == OPTION_THREE)
		{
			sr_ecp_msg->message("Wyznaczanie modelu metoda alternatywna y1");
			procedure_type = ALTERNATIVE_Y_METHOD_1;
		}
		else if (option == OPTION_FOUR)
		{
			sr_ecp_msg->message("Wyznaczanie modelu metoda alternatywna y2");
			procedure_type = ALTERNATIVE_Y_METHOD_2;
		}

	}
	else if(option == OPTION_THREE)
	{
		automatic = true;
	}
	else if(option == OPTION_FOUR)
	{
		correction = true;
	}
	// set_trajectory(robot, procedure_type);

	// w domu zastanowic sie nad jakims sprytnym uporzadkowaniem tego
	// bo aktualnie nie ma jeszcze przeciez korekcji
	if(correction)
	{
		// przerobic to bo korekcja modelu moze miec sens tylko po jego wyznaczeniu
		// ustawienie odpowiednich trajektorii
		set_correction_trajectories();
		double test_model[] = {1,2,3,4};
		model_correction(test_model);
	}
	
	int T;
	if(automatic)
	{
		T = 5;
		//procedure_type = 0;
		cout<<"T: "<<T<<endl;
	}
	else
		T = 1;
		
	int map_tab[] = {0,1,3,2,4};
	int i = 0;
	int count = 4;
	while(true)
	{
		if(automatic)
			procedure_type = map_tab[i];
		cout<<"procedure type: "<<procedure_type<<endl;
		set_trajectory(robot, procedure_type);
		if(procedure_type == STANDARD)
		{   
			str<<"STANDARD"<<endl;
			method_standard(count);
		}
		else if(procedure_type == ALTERNATIVE_X_METHOD_1)
		{
			str<<"ALTERNATIVE_X_METHOD_1"<<endl;
			int sequence[] = {0,1}; 
			method_alternative(0,sequence,count);
		}
		else if(procedure_type == ALTERNATIVE_X_METHOD_2)
		{
			str<<"ALTERNATIVE_X_METHOD_2"<<endl;
			int sequence[] = {0,1};
			int sequence_reverse[] = {1,0}; 
			method_alternative(0,sequence,count);
			method_alternative(0,sequence_reverse,count);
		}
		else if(procedure_type == ALTERNATIVE_Y_METHOD_1)
		{
			str<<"ALTERNATIVE_Y_METHOD_1"<<endl;
			int sequence[] = {0,1}; 
			method_alternative(1,sequence,count);
		}
		else if(procedure_type == ALTERNATIVE_Y_METHOD_2)
		{
			str<<"ALTERNATIVE_Y_METHOD_2"<<endl;
			int sequence[] = {0,1};
			int sequence_reverse[] = {1,0}; 
			method_alternative(1,sequence,count);
			method_alternative(1,sequence_reverse,count);
		}
		
		if(automatic)
			i++;
		T--;
		if(T == 0)
			break;
	}
	ecp_termination_notice();
	ecp_wait_for_stop();
	
std::cout<<"end\n"<<std::endl;
};

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_tzu_fs(_config);
};

void ecp_task_tzu_fs::method_alternative(int type, int sequence[], int T)
{
	for(int i = 0 ; i < T ; i++)
	{
		while(true)			
		{	
			str<<"->pomiar: "<<i<<endl;
			ftcg->Move();
			tcg->set_tool_parameters(0,0,0);
			tcg->Move();
			sg->load_file_with_path(trajectories[sequence[0]]);
			sg->Move ();		
			fmg->Move();
			cout<<"pomiar alternative1: "<<fmg->weight<<endl;
			//str<<fmg->weight<<endl;
			befg->Move();
			sg->load_file_with_path(trajectories[sequence[1]]);
			sg->Move ();		
			fmg->Move();
			cout<<"pomiar alternative2: "<<fmg->weight<<endl;
			str<<fmg->weight<<endl;
			weight = (-(fmg->weight[type]))/2;
			cout<<"weight: "<<weight<<endl;
			str<<weight<<endl;
//			ecp_termination_notice();
//			ecp_wait_for_stop();
			break;
		}
	}
}

void ecp_task_tzu_fs::method_standard(int T)
{
	double weight_s = 0;
	double P_x_s = 0;
	double P_y_s = 0;
	double P_z_s = 0;
	for(int i = 0 ; i < T ; i++)
	{
		while(true)			
		{	
			str<<"->pomiar: "<<i<<endl; 
			// ETAP PIERWSZY - chwytka skierowany pionowo do dolu, biasowanie odczytow, 
			// zmiana narzedzia kinematycznego, ustawienie end-effector frame E jako sensor frame S
			sg->load_file_with_path(trajectories[TRAJECTORY_VERTICAL_DOWN]);
			sg->Move ();		
			sleep(2);
			befg->Move();
			ftcg->Move();
			tcg->set_tool_parameters(0,0,0);
			tcg->Move();
			// ETAP DRUGI - chwytak skierowany pionowo do gory, odczyt i obliczenie trzech pierwszych parametrow
			// wagi, parametrow translacji?!?
			sg->load_file_with_path(trajectories[TRAJECTORY_VERTCAL_UP]);
			sg->Move ();
			// fmg->Move();
			force_measurrement();
			// weight = (fmg->weight[FORCE_Z])/2;
			weight = (pom[FORCE_Z])/2;
			P_x = -pom[TORQUE_Y]/(2*weight);
			P_y = pom[TORQUE_X]/(2*weight);
			// P_x = -fmg->weight[TORQUE_Y]/(2*weight);
			// P_y = fmg->weight[TORQUE_X]/(2*weight);
			// ETAP TRZECI - chwytak skierowany horyzontalnie, obliczenie ostatniego z parametrów modelu
			sg->load_file_with_path(trajectories[TRAJECTORY_HORIZONTAL]);
			sg->Move ();
			// fmg->Move();
			force_measurrement();
			// P_z = fmg->weight[TORQUE_Y]/weight + P_x;
			P_z = pom[TORQUE_Y]/weight + P_x;
			
			cout<<"Parametry modelu srodka ciezkosci narzedzia"<<endl
				<<"weight: "<<weight<<endl<<"P_x: "<<P_x<<endl<<"P_y: "<<P_y<<endl<<"P_z: "<<P_z<<endl; 
			str<<"Parametry modelu srodka ciezkosci narzedzia"<<endl
				<<"weight: "<<weight<<endl<<"P_x: "<<P_x<<endl<<"P_y: "<<P_y<<endl<<"P_z: "<<P_z<<endl;
			sleep(1);
			weight_s += weight;
			P_x_s += P_x;
			P_y_s += P_y;
			P_z_s += P_z;
			break;
		}
	}
	cout<<"Parametry modelu srodka ciezkosci narzedzia srednio"<<endl
				<<"weight: "<<weight_s/T<<endl<<"P_x: "<<P_x_s/T<<endl<<"P_y: "<<P_y_s/T<<endl<<"P_z: "<<P_z_s/T<<endl;
}

void ecp_task_tzu_fs::set_trajectory(int robot_type, int procedure_type)
{
	if((robot_type == POSTUMENT) && (procedure_type == STANDARD))
	{
		trajectories[0] = "../trj/tzu/standard/postument/tzu_1_postument.trj";
		trajectories[1] = "../trj/tzu/standard/postument/tzu_2_postument.trj";
		trajectories[2] = "../trj/tzu/standard/postument/tzu_3_postument.trj";
	}
	else if((robot_type == POSTUMENT) && (procedure_type == ALTERNATIVE_X_METHOD_1))
	{
		trajectories[0] = "../trj/tzu/alternative/postument/x_weight_meassure/method_1/tzu_1_postument.trj";
		trajectories[1] = "../trj/tzu/alternative/postument/x_weight_meassure/method_1/tzu_2_postument.trj";
	}
	else if((robot_type == POSTUMENT) && (procedure_type == ALTERNATIVE_X_METHOD_2))
	{
		trajectories[0] = "../trj/tzu/alternative/postument/x_weight_meassure/method_2/tzu_1_postument.trj";
		trajectories[1] = "../trj/tzu/alternative/postument/x_weight_meassure/method_2/tzu_2_postument.trj";
	}
	else if((robot_type == POSTUMENT) && (procedure_type == ALTERNATIVE_Y_METHOD_1))
	{
		trajectories[0] = "../trj/tzu/alternative/postument/y_weight_meassure/method_1/tzu_1_postument.trj";
		trajectories[1] = "../trj/tzu/alternative/postument/y_weight_meassure/method_1/tzu_2_postument.trj";
	}
	else if((robot_type == POSTUMENT) && (procedure_type == ALTERNATIVE_Y_METHOD_2))
	{
		trajectories[0] = "../trj/tzu/alternative/postument/y_weight_meassure/method_2/tzu_1_postument.trj";
		trajectories[1] = "../trj/tzu/alternative/postument/y_weight_meassure/method_2/tzu_2_postument.trj";
	}
	
	else if((robot_type == ON_TRACK) && (procedure_type == STANDARD))
	{
		trajectories[0] = "../trj/tzu/standard/on_track/tzu_1_on_track.trj";
		trajectories[1] = "../trj/tzu/standard/on_track/tzu_2_on_track.trj";
		trajectories[2] = "../trj/tzu/standard/on_track/tzu_3_on_track.trj";
	}
	else if((robot_type == ON_TRACK) && (procedure_type == ALTERNATIVE_X_METHOD_1))
	{
		trajectories[0] = "../trj/tzu/alternative/on_track/x_weight_meassure/method_1/tzu_1_on_track.trj";
		trajectories[1] = "../trj/tzu/alternative/on_track/x_weight_meassure/method_1/tzu_2_on_track.trj";
	}
	else if((robot_type == ON_TRACK) && (procedure_type == ALTERNATIVE_X_METHOD_2))
	{
		trajectories[0] = "../trj/tzu/alternative/on_track/x_weight_meassure/method_2/tzu_1_on_track.trj";
		trajectories[1] = "../trj/tzu/alternative/on_track/x_weight_meassure/method_2/tzu_2_on_track.trj";
	}   
	else if((robot_type == ON_TRACK) && (procedure_type == ALTERNATIVE_Y_METHOD_1))
	{
		trajectories[0] = "../trj/tzu/alternative/on_track/y_weight_meassure/method_1/tzu_1_on_track.trj";
		trajectories[1] = "../trj/tzu/alternative/on_track/y_weight_meassure/method_1/tzu_2_on_track.trj";
	}
	else if((robot_type == ON_TRACK) && (procedure_type == ALTERNATIVE_Y_METHOD_2))
	{
		trajectories[0] = "../trj/tzu/alternative/on_track/y_weight_meassure/method_2/tzu_1_on_track.trj";
		trajectories[1] = "../trj/tzu/alternative/on_track/y_weight_meassure/method_2/tzu_2_on_track.trj";
	}
}

void ecp_task_tzu_fs::set_correction_trajectories() // mozna wywalic zmienna robot z klasy i wtedy jawnie przekazywac ja tu do funkcji
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

int ecp_task_tzu_fs::czy_miesci_sie_w_zakladanej_dokladnosci(double dokladnosc, double pomiar)
{
	// zero jest "pozycja" od ktorej maja odiegac dane pomiary
	if(pomiar >= dokladnosc)		// zastanowic sie czy nie zwracac tu jakiejs odleglosci bledu
		return 1;
	else if(pomiar >= -dokladnosc)
		return -1;
	return 0;
}
void ecp_task_tzu_fs::model_correction(double model[])
{
	double zadana_dokladnosc = 0.5; // czyli maksymalne odchylenie od zera w tym przypadku ma wynosci maksymalnie +/- 0.5
	double krok_korekty = 0.01;
	int ilosc_krokow_korekcyjnych = 10;
	while(true)			
	{
		cout<<"START MODEL CORRECTION"<<endl;
		// befg->Move();
		cout<<"Biasowanie dokonane"<<endl;
//		tcg->set_tool_parameters(0,0,0.09); // spytac sie jak to jest dokladnie z ustawianiem tego przesuniecia u jakie powinno ono byc w tym przypadku
//		tcg->Move();
		cout<<"Rozpoczecie procedur korekcyjnych dla roznych trajektorii"<<endl;
		
		for(int i = 0 ; i < NUMBER_OF_TEST_TRAJECTORIES ; i++)
		{
			sg->load_file_with_path(test_trajectories[i]);
			sg->Move();
			fmg->Move();
			cout<<"pomiar korekcyjny "<<i<<": "<<fmg->weight<<endl;
			str<<"pomiar korekcyjny "<<i<<": "<<fmg->weight<<endl;
			// jak to jest z korekcja gdy odkladaja nam sie momenty sil
			// akualnie testy/przemyslenia jedynie dla sil
			for(int j = 0 ; j < 3 ; j++)
			{
				
				for(int k = 0 ; k < ilosc_krokow_korekcyjnych ; k++)
				{
					int kor = czy_miesci_sie_w_zakladanej_dokladnosci(zadana_dokladnosc,fmg->weight[i]);
					if(kor != 0) // czy korygowac
					{
						// korygowac
						if(kor > 0)
						{
							model[i] -= krok_korekty;
						}
						else
						{
							model[i] += krok_korekty;
						}
					}
					else
						break; // nie korygujemy
				}
			}
		}
		break;
	}
}

char*ecp_task_tzu_fs::get_trajectory(double x[])
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
/*
bool force_meassure_generator::next_step()
{
	//Ft_v_vector average_meassurement[meassurement_count];
	sleep(2);
	for(int i = 0 ; i < meassurement_count ; i++)
	{
		Homog_matrix current_frame_wo_offset(the_robot->EDP_data.current_arm_frame);
		current_frame_wo_offset.remove_translation();
	
		Ft_v_vector force_torque(the_robot->EDP_data.current_force_xyz_torque_xyz);
		weight += force_torque;
		weight = force_torque;
		sleep(sleep_time);
	}
	return false;
}
*/
bool force_meassure_generator::next_step()
{
//	cout<<"sleep time: "<<sleep_time<<endl;
//	cout<<"meassurement_count: "<<meassurement_count<<endl;
//	sleep(sleep_time);
	for(int i = 0 ; i < meassurement_count ; i++)
	{
		Homog_matrix current_frame_wo_offset(the_robot->EDP_data.current_arm_frame);
		current_frame_wo_offset.remove_translation();
	
		Ft_v_vector force_torque(the_robot->EDP_data.current_force_xyz_torque_xyz);
		weight += force_torque;
		weight = force_torque;
		usleep(50000);
//		sleep(1);
//		cout<<"force torque: "<<force_torque<<endl;
	}
	
//	cout<<"weight przed: "<<weight<<endl;
	for(int i = 0 ; i < 6 ; i++)
		weight[i] = weight[i]/meassurement_count;
//	cout<<"weight po: "<<weight<<endl;
	return false;
}
