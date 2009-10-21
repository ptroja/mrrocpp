#include <iostream>
#include <fstream>
#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp_mp/ecp_mp_s_force.h"
#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/common/ecp_t_tzu_fs.h"
#include "lib/mathtr.h"
#include "ecp/common/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

/*
 * przy najblizszej okazji zrobic tu porzadek przy wykorzystaniu jakiegos porzadnego edytora!!!
 */
using namespace std;
/** konstruktor konstruktor**/
tzu_fs::tzu_fs(lib::configurator &_config) : task(_config)
{
	if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
	{
		ecp_m_robot = new irp6ot::robot (*this);
		robot = ON_TRACK;
		str.open("../on_track_results.txt");
	}
	else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
	{
		ecp_m_robot = new irp6p::robot (*this);
		robot = POSTUMENT;
		str.open("../postument_results.txt");
	}

	sg = new generator::smooth (*this, true, false);
	befg = new generator::bias_edp_force(*this);
	fmg = new generator::force_meassure_generator(*this,100000,20);
	ftcg = new generator::force_tool_change(*this);
	tcg = new generator::tool_change(*this,true);
	etnrg = new generator::tff_nose_run(*this,8);
	sr_ecp_msg->message("ECP loaded");
}

void tzu_fs::main_task_algorithm(void)
{
	bool automatic = false;
	int procedure_type;

	int option = choose_option ("1 - Standard, 2 - Alternative, 3 - Auto", 3);
	if (option == lib::OPTION_ONE)
    {
    	sr_ecp_msg->message("Wyznaczanie modelu metoda standardowa");
   		procedure_type = STANDARD;
   	}
    else if (option == lib::OPTION_TWO)
    {
    	// aktualnie wyznaczany jest tu tylko ci��ar narz�dzia, ale pomy�le� r�wnie� nad tym by w r��ny spos�b wyznaczy� ca�y model
		sr_ecp_msg->message("Wyznaczanie modelu metoda alternatywna x");
		option = choose_option ("1 - x1, 2 - x2, 3 - y1, 4 - y2", 4);
		if (option == lib::OPTION_ONE)
		{
			sr_ecp_msg->message("Wyznaczanie modelu metoda alternatywna x1");
			procedure_type = ALTERNATIVE_X_METHOD_1;
		}
		else if (option == lib::OPTION_TWO)
		{
			sr_ecp_msg->message("Wyznaczanie modelu metoda alternatywna x2");
			procedure_type = ALTERNATIVE_X_METHOD_2;
		}
		else if (option == lib::OPTION_THREE)
		{
			sr_ecp_msg->message("Wyznaczanie modelu metoda alternatywna y1");
			procedure_type = ALTERNATIVE_Y_METHOD_1;
		}
		else if (option == lib::OPTION_FOUR)
		{
			sr_ecp_msg->message("Wyznaczanie modelu metoda alternatywna y2");
			procedure_type = ALTERNATIVE_Y_METHOD_2;
		}
	}
	else if(option == lib::OPTION_THREE)
	{
		automatic = true;
	}

	int T;
	if(automatic) //przechodzimy przez wszystkit metody
		T = 5;
	T = 1;

	int map_tab[] = {0,1,3,2,4};
	int i = 0;
	int count = 10;
	while(true)
	{
		if(automatic)
			procedure_type = map_tab[i];
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
}

task* return_created_ecp_task (lib::configurator &_config)
{
	return new tzu_fs(_config);
}

void tzu_fs::method_alternative(int type, int sequence[], int T)
{
	for(int i = 0 ; i < T ; i++)
	{
		while(true)
		{
			ftcg->Move();
			tcg->set_tool_parameters(0,0,0);
			tcg->Move();
			sg->load_file_with_path(trajectories[sequence[0]]);
			sg->Move ();
			fmg->Move();
			cout<<"pomiar alternative1: "<<fmg->get_meassurement()<<endl;
			str<<"pomiar alternative1: "<<fmg->get_meassurement()<<endl;
			sleep(2);
			befg->Move();
			sg->load_file_with_path(trajectories[sequence[1]]);
			sg->Move ();
			fmg->Move();
			cout<<"pomiar alternative2: "<<fmg->get_meassurement()<<endl;
			str<<"pomiar alternative2: "<<fmg->get_meassurement()<<endl;
			weight = (-(fmg->get_meassurement()[type]))/2;
			cout<<"weight: "<<weight<<endl;
			str<<"weight: "<<weight<<endl;
			break;
		}
	}
}

void tzu_fs::method_standard(int T)
{
	double weight_s = 0;
	double P_x_s = 0;
	double P_y_s = 0;
	double P_z_s = 0;
	for(int i = 0 ; i < T ; i++)
	{
		while(true)
		{
			// ETAP PIERWSZY - chwytka skierowany pionowo do dolu, biasowanie odczytow,
			// zmiana narzedzia kinematycznego, ustawienie end-effector frame E jako sensor frame S
			sg->load_file_with_path(trajectories[TRAJECTORY_VERTICAL_DOWN]);
			sg->Move ();
//			fmg->Move();
//			cout<<"wynik1: "<< fmg->get_meassurement() << endl;
			sleep(2);
			befg->Move();
			ftcg->Move();
			tcg->set_tool_parameters(0,0,0);
			tcg->Move();
	//		fmg->Move();
	//		cout<<"wynik2: "<< fmg->get_meassurement() << endl;
			// ETAP DRUGI - chwytak skierowany pionowo do gory, odczyt i obliczenie trzech pierwszych parametrow
			// wagi, oraz przesuniecia wzdluz osi x i y
			sg->load_file_with_path(trajectories[TRAJECTORY_VERTCAL_UP]);
			sg->Move ();
			fmg->Move();
			//cout<<"wynik3: "<< fmg->get_meassurement() << endl;
			weight = (fmg->get_meassurement()[FORCE_Z])/2;
			P_x = -fmg->get_meassurement()[TORQUE_Y]/(2*weight);
			P_y = fmg->get_meassurement()[TORQUE_X]/(2*weight);
			// ETAP TRZECI - chwytak skierowany horyzontalnie, obliczenie ostatniego z parametr�w modelu, przesuniecia wzdluz osi z
			sg->load_file_with_path(trajectories[TRAJECTORY_HORIZONTAL]);
			sg->Move ();
			fmg->Move();
			P_z = fmg->get_meassurement()[TORQUE_Y]/weight + P_x;

			cout<<"Parametry modelu srodka ciezkosci narzedzia"<<endl
				<<"weight: "<<weight<<endl<<"P_x: "<<P_x<<endl<<"P_y: "<<P_y<<endl<<"P_z: "<<P_z<<endl;
			str<<"Parametry modelu srodka ciezkosci narzedzia"<<endl
				<<"weight: "<<weight<<endl<<"P_x: "<<P_x<<endl<<"P_y: "<<P_y<<endl<<"P_z: "<<P_z<<endl;
			weight_s += weight;
			P_x_s += P_x;
			P_y_s += P_y;
			P_z_s += P_z;
			break;
		}
	}
	cout<<"Parametry modelu srodka ciezkosci narzedzia srednio"<<endl
		<<"weight: "<<weight_s/T<<endl<<"P_x: "<<P_x_s/T<<endl<<"P_y: "<<P_y_s/T<<endl<<"P_z: "<<P_z_s/T<<endl;
	str<<"Parametry modelu srodka ciezkosci narzedzia srednio"<<endl
		<<"weight: "<<weight_s/T<<endl<<"P_x: "<<P_x_s/T<<endl<<"P_y: "<<P_y_s/T<<endl<<"P_z: "<<P_z_s/T<<endl;
}

void tzu_fs::set_trajectory(int robot_type, int procedure_type)
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

const char*tzu_fs::get_trajectory(double x[])
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

} // namespace task

namespace generator {

/**** force meassure generator ****/

/** konstruktor **/
force_meassure_generator::force_meassure_generator(common::task::task& _ecp_task, int _sleep_time, int _meassurement_count) :
	generator(_ecp_task)
{
	sleep_time = _sleep_time;
	meassurement_count = _meassurement_count;
	init_meassurement_count = _meassurement_count;
}

/** ustawienie konfiguracji generatora **/
void force_meassure_generator::set_configuration(int _sleep_time, int _meassurement_count)
{
	sleep_time = _sleep_time;
	meassurement_count = _meassurement_count;
	init_meassurement_count = _meassurement_count;
}

/** first step **/
bool force_meassure_generator::first_step()
{
	the_robot->EDP_data.instruction_type = lib::GET;
 	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.get_arm_type = lib::FRAME;
	the_robot->EDP_data.next_interpolation_type
			= lib::TCIM;
	for(int i = 0; i < 6 ; i++)
		weight[i] = 0;

	return true;
}

/** next step **/
bool force_meassure_generator::next_step()
{
	usleep(sleep_time);
	lib::Homog_matrix current_frame_wo_offset(the_robot->EDP_data.current_arm_frame);
	current_frame_wo_offset.remove_translation();

	lib::Ft_v_vector force_torque(the_robot->EDP_data.current_force_xyz_torque_xyz);
	weight += force_torque;

//	cout<<"force_torque: "<<force_torque<<endl;
	meassurement_count--;
	if(meassurement_count <= 0)
	{
	//cout<<"in gen mes: "<<weight<<endl;
		meassurement_count = init_meassurement_count;
		for(int i = 0 ; i < 6 ; i++)
			weight[i] = weight[i]/meassurement_count;
		return false;
	}
	return true;
}

lib::Ft_v_vector& force_meassure_generator::get_meassurement()
{
//cout<<"in gen mes bef: "<<weight<<endl;
//	for(int i = 0 ; i < 6 ; i++)
//		weight[i] = weight[i]/meassurement_count;
//cout<<"in gen mes aft: "<<weight<<endl;
	return weight;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
