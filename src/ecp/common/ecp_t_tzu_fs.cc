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

using namespace std;
/** konstruktor **/
ecp_task_tzu_cs_irp6ot::ecp_task_tzu_cs_irp6ot(configurator &_config) : ecp_task(_config)
{
	sg = NULL;
	befg = NULL;
	ftcg = NULL;
	tcg = NULL;
	etnrg = NULL;
	str.open("../results.txt",ios::app);
};

/** destruktor **/
ecp_task_tzu_cs_irp6ot::~ecp_task_tzu_cs_irp6ot()
{
	str<<"--- KONIEC ---"<<endl;
	str.close();
};


// methods for ECP template to redefine in concrete classes
void ecp_task_tzu_cs_irp6ot::task_initialization(void) 
{
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
	
	sg = new ecp_smooth_generator (*this, true, false);
	befg = new bias_edp_force_generator(*this);
	fmg = new force_meassure_generator(*this);	
	ftcg = new ecp_force_tool_change_generator(*this);
	tcg = new ecp_tool_change_generator(*this,true);
	etnrg = new ecp_tff_nose_run_generator(*this,8);
	sr_ecp_msg->message("ECP loaded");
};

void ecp_task_tzu_cs_irp6ot::main_task_algorithm(void)
{
	bool automatic = false;
	sr_ecp_msg->message("ECP cs irp6ot  - pushj start in tzu");
//	str.open("../results.txt");
//	str<<"test dupa"<<endl;
//	str.close();
	ecp_wait_for_start();
	int procedure_type;
	char* additional_move;
	
	if(robot == ON_TRACK)
		additional_move = "../trj/tzu/tzu_on_track_1.trj";
	else
		additional_move = "../trj/tzu/tzu_postument_1.trj";

	int option = choose_option ("1 - Metoda standardowa, 2 - Metody alternatywne, 3 - Auitomat", 3);
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
	
	// set_trajectory(robot, procedure_type);

	int T;
	if(automatic)
	{
		T = 5;
		//procedure_type = 0;
		cout<<"T: "<<T<<endl;
	}
	else
		T = 1;
		
	// trzeba stworzyc tablice mapujaca kolejnosc wykonywanych ruchow
	int map_tab[] = {0,1,3,2,4};
	int i = 0;
	int count = 2;
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
	return new ecp_task_tzu_cs_irp6ot(_config);
};

void ecp_task_tzu_cs_irp6ot::method_alternative(int type, int sequence[], int T)
{
	for(int i = 0 ; i < T ; i++)
	{
		while(true)			
		{	
			str<<"->pomiar: "<<i<<endl;
			ftcg->Move();
			tcg->set_tool_parameters(0,0,0.09);
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

void ecp_task_tzu_cs_irp6ot::method_standard(int T)
{
	for(int i = 0 ; i < T ; i++)
	{
		while(true)			
		{	
			str<<"->pomiar: "<<i<<endl; 
			// ETAP PIERWSZY - chwytka skierowany pionowo do dolu, biasowanie odczytow, 
			// zmiana narzedzia kinematycznego, ustawienie end-effector frame E jako sensor frame S
			sg->load_file_with_path(trajectories[TRAJECTORY_VERTICAL_DOWN]);
			sg->Move ();		
			befg->Move();
			ftcg->Move();
			tcg->set_tool_parameters(0,0,0.09);
			tcg->Move();
		
			// ETAP DRUGI - chwytak skierowany pionowo do gory, odczyt i obliczenie trzech pierwszych parametrow
			// wagi, parametrow translacji?!?
			sg->load_file_with_path(trajectories[TRAJECTORY_VERTCAL_UP]);
			sg->Move ();
			fmg->Move();
			weight = (-(fmg->weight[FORCE_Z]))/2;
			P_x = fmg->weight[TORQUE_Y]/(2*weight);
			P_y = -fmg->weight[TORQUE_X]/(2*weight);
			// cout<<"torque_x: "<<fmg->weight[TORQUE_X]<<endl;
			// cout<<"torque_y: "<<fmg->weight[TORQUE_Y]<<endl;
			// cout<<"test funkcji: "<<*(fmg->get_meassurement())<<endl;
	/*		
	 		for(int i = 0 ; i < 10 ; i++)
			{
				fmg->Move();
				//weight = fmg->weight[2]/2;
				std::cout<<"pomiar 1: "<<fmg->weight<<std::endl;
				std::cout<<"weight_1"<<": "<<fmg->weight[2]/2<<std::endl;
				sleep(1);
			}
	*/
			// ETAP TRZECI - chwytak skierowany horyzontalnie, obliczenie ostatniego z parametrów modelu
			sg->load_file_with_path(trajectories[TRAJECTORY_HORIZONTAL]);
			
			sg->Move ();
			fmg->Move();
	
			P_z = -fmg->weight[TORQUE_Y]/weight;
			// cout<<"torque_y: "<<fmg->weight[TORQUE_Y]<<endl;
	/*		
	 		for(int i = 0 ; i < 10 ; i++)
			{
				fmg->Move();
				std::cout<<"pomiar 2: "<<fmg->weight<<std::endl;
				std::cout<<"t1: "<<-(fmg->weight[4]/weight)<<std::endl;
				sleep(1);
			}
	*/
			cout<<"Parametry modelu srodka ciezkosci narzedzia"<<endl
				<<"weight: "<<weight<<endl<<"P_x: "<<P_x<<endl<<"P_y: "<<P_y<<endl<<"P_z: "<<P_z<<endl; 
			str<<"Parametry modelu srodka ciezkosci narzedzia"<<endl
				<<"weight: "<<weight<<endl<<"P_x: "<<P_x<<endl<<"P_y: "<<P_y<<endl<<"P_z: "<<P_z<<endl;
			// test nose - start
			sleep(1);
			// test
	///		set_test_trajectory(robot);
	///		sg->load_file_with_path(test_trajectories[0]);
	///		sg->Move();
			// test
	///		sg->load_file_with_path(additional_move);
	///		sg->Move();
	//		befg->Move();
			
			// x,y,z,weight		
			// ustawienia zgodne z plikiem konfiguracyjnym
	//		ftcg->set_tool_parameters(0.004,0.0,0.066,10.8);
	//		ftcg->Move();
	//		fmg->Move();
	//		cout<<"pomiar1: "<<fmg->weight<<endl;
			
	//		befg->Move();
			// ustawienie wyznaczonego modelu i procedury testowania go
	///		ftcg->set_tool_parameters(P_x,P_y,P_z,weight);
	///		ftcg->Move();
			
	///		befg->Move();	
	///		etnrg->set_force_meassure(true);
	///		etnrg->Move();
			// koniec testowania przy pomocy nose generatora
			
			// testy polegajace na ustawianiu manipulatora w roznych pozycjach i sprawdzaniu uzyskanych odczytow sily
			// na 6 joint obrot
	//		fmg->Move();
	//		cout<<"pomiar2: "<<fmg->weight<<endl;
	//		set_test_trajectory(robot);
	//		sg->load_file_with_path(test_trajectories[0]);
	//		sg->Move();
	//		fmg->Move();
	//		cout<<"pomiar3: "<<fmg->weight<<endl;
			// koniec testow
			break;
		}
	}
}

void ecp_task_tzu_cs_irp6ot::set_trajectory(int robot_type, int procedure_type)
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

void ecp_task_tzu_cs_irp6ot::set_test_trajectory(int robot_type)
{
/*
 	if(robot_type == POSTUMENT)
	{
		test_trajectories[0] = "../trj/tzu/tzu_1_postument_test.trj";
		test_trajectories[1] = "../trj/tzu/tzu_2_postument_test.trj";
	}
	else
	{
		test_trajectories[0] = "../trj/tzu/tzu_1_on_track_test.trj";
		test_trajectories[1] = "../trj/tzu/tzu_2_on_track_test.trj";
	}
*/
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
bool force_meassure_generator::next_step()
{
	//Ft_v_vector average_meassurement[meassurement_count];
	sleep(2);
	for(int i = 0 ; i < meassurement_count ; i++)
	{
		//cout<<"pomiar: "<<weight<<endl; 
		Homog_matrix current_frame_wo_offset(the_robot->EDP_data.current_arm_frame);
		current_frame_wo_offset.remove_translation();
	
		Ft_v_vector force_torque(the_robot->EDP_data.current_force_xyz_torque_xyz);
		weight += force_torque;
		weight = force_torque;
		sleep(sleep_time);
	}
//	double test1 = 22.2;
//	int test2 = 2;
//	cout<<"dzielenie: "<<test1/test2<<endl;

//	for(int i = 0 ; i < 6 ; i++)
//		weight[i] = weight[i]/meassurement_count;
	return false;
}

