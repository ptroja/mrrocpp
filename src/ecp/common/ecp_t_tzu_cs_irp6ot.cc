#include <iostream>
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
#include "ecp/common/ecp_t_tzu_cs_irp6ot.h"
#include "lib/mathtr.h"
#include "ecp/common/ecp_g_smooth.h"

#define WEIGHT_FOR 10
#define T1_FOR 10
#define T2_FOR 10
#define T3_FOR 10

/** konstruktor **/
ecp_task_tzu_cs_irp6ot::ecp_task_tzu_cs_irp6ot(configurator &_config) : ecp_task(_config)
{
	sg = NULL;
	befg = NULL;
	ftcg = NULL;
	tcg = NULL;
};

/** destruktor **/
ecp_task_tzu_cs_irp6ot::~ecp_task_tzu_cs_irp6ot()
{
};


// methods for ECP template to redefine in concrete classes
void ecp_task_tzu_cs_irp6ot::task_initialization(void) 
{
//	cout<<"->inicjalizacja robota"<<endl;
	// ecp_m_robot = new ecp_irp6_on_track_robot (*this);
	if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
	{
		ecp_m_robot = new ecp_irp6_on_track_robot (*this);
		trajectories[1] = "../trj/tzu/tzu_1_on_track.trj";
		trajectories[2] = "../trj/tzu/tzu_2_on_track.trj";
		trajectories[3] = "../trj/tzu/tzu_3_on_track.trj";
		trajectories[4] = "../trj/tzu/tzu_4_on_track.trj";
	}
	else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
	{
		ecp_m_robot = new ecp_irp6_postument_robot (*this);
		trajectories[1] = "../trj/tzu/tzu_1_postument.trj";
		trajectories[2] = "../trj/tzu/tzu_2_postument.trj";
		trajectories[3] = "../trj/tzu/tzu_3_postument.trj";
		trajectories[4] = "../trj/tzu/tzu_4_postument.trj";
	}
	
	sg = new ecp_smooth_generator (*this, true, false);
	befg = new bias_edp_force_generator(*this);
	wmg = new weight_meassure_generator(*this, 1);
	fmg = new force_meassure_generator(*this, Z_FORCE_MEASSURE);	
	ftcg = new ecp_force_tool_change_generator(*this);
	tcg = new ecp_tool_change_generator(*this,true);
// to chyba nie bedzie potrzebne, zamiast tego bedzie mozna wykorzystac zrobiony prostsza metoda gotowy generator 
//	short use_force_sensor = config.return_int_value("use_force_sensor");
//    if (use_force_sensor == 1)
//	{
//		cout<<"uzywamy czujnika sily"<<endl;
//        	sr_ecp_msg->message("Using force sensor for move control");
//         	// Stworzenie obiektu czujnik.
//		// ini_con->create_vsp ("[vsp_fs]");
//		sensor_m[SENSOR_FORCE_ON_TRACK] = new ecp_mp_force_sensor(SENSOR_FORCE_ON_TRACK, "[vsp_fs]", *this);
//		// Konfiguracja czujnika.
//		sensor_m[SENSOR_FORCE_ON_TRACK]->configure_sensor();
//         	// Stworzenie listy czujnikow -> glowa = (czujnik sily).
//        	//fctg->sensor_m[SENSOR_FORCE_ON_TRACK] = sensor_m[SENSOR_FORCE_ON_TRACK];
//         	// Odczyt wielkosci niebezpiecznej sily z pliku INI.
//         	//fctg->set_dangerous_force();
//		cout<<"czujnik skonfigurowany"<<endl;
//	}
//	else
//	{
//		cout<<"nie uzywamy czujnika sily"<<endl;
//          sr_ecp_msg->message("Not using force sensor for move control");
//          // Pusta lista czujnikow.
//          //fctg->sensor_m.clear();
//          sensor_m.clear();
//          // Pusty czujnik.
//	};  // end: else
	sr_ecp_msg->message("ECP loaded");
};

void ecp_task_tzu_cs_irp6ot::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP cs irp6ot  - pushj start in tzu");
	ecp_wait_for_start();
	
	/* stworzenie generatora ruchu */
	// sprawdzic czy drugi argument jest rzeczywiscie potrzebny
	//tzu_simple_generator sg(*this, 8);
	// sprawdzic zmienna sensor_m
	//sg.sensor_m = sensor_m;
	
	//trzeba jeszcze stworzy generatory ktore zajma sie odczytaniem sily
	while(true)			
	{ 
		double weight;
		double torque_xc[6];
		double torque_zd[6];
		double torque_ze[6];
		double t_z;
		double t_y;
		double t_x;
		
		// ETAP PIERWSZY
		sg->load_file_with_path(trajectories[TRAJECTORY_1]);
		sg->Move ();		
		befg->Move();
		ftcg->Move();
		tcg->set_tool_parameters(0,0,0.09);
		tcg->Move();
		
		// ETAP DRUGI
		sg->load_file_with_path(trajectories[TRAJECTORY_2]);
		sg->Move ();
//		fmg->Move();
//		weight = fmg->get_meassurement()/2;

//		std::cout<<"pomiar 1: "<<fmg->weight<<std::endl;
//		weight = fmg->weight[2]/2;
//		std::cout<<"weight: "<<fmg->weight[2]/2<<std::endl;
		for(int i = 0 ; i < WEIGHT_FOR ; i++)
		{
			fmg->Move();
			weight = fmg->weight[2]/2;
			std::cout<<"pomiar 1: "<<fmg->weight<<std::endl;
			std::cout<<"weight_1"<<": "<<fmg->weight[2]/2<<std::endl;
			sleep(1);
		}
		
		
		// ETAP TRZECI
		sg->load_file_with_path(trajectories[TRAJECTORY_3]);
		sg->Move ();
		fmg->change_meassurement(X_TORQUE_MEASSURE);
//		fmg->Move();
//		torque_xc = fmg->get_meassurement()/weight;
//		std::cout<<"pomiar 2: "<<fmg->weight<<std::endl;
//		std::cout<<"t1: "<<-(fmg->weight[4]/weight)<<std::endl;
		for(int i = 0 ; i < T1_FOR ; i++)
		{
			fmg->Move();
			std::cout<<"pomiar 2: "<<fmg->weight<<std::endl;
			std::cout<<"t1: "<<-(fmg->weight[4]/weight)<<std::endl;
			sleep(1);
		}
		// ETAP CZWARTY
		befg->Move();
		sg->load_file_with_path(trajectories[TRAJECTORY_4]);
		sg->Move ();
		fmg->change_meassurement(Z_TORQUE_MEASSURE);
//		fmg->Move();
//		torque_zd = fmg->get_meassurement()/weight;
//		std::cout<<"pomiar 3: "<<fmg->weight<<std::endl;
//		std::cout<<"t2: "<<fmg->weight[5]/weight<<std::endl;
		for(int i = 0 ; i < T2_FOR ; i++)
		{
			fmg->Move();
			std::cout<<"pomiar 3: "<<fmg->weight<<std::endl;
			std::cout<<"t2: "<<fmg->weight[5]/weight<<std::endl;
			sleep(1);
		}
		
		// ETAP PIATY
		befg->Move();
		sg->load_file_with_path(trajectories[TRAJECTORY_3]);
		sg->Move ();
		fmg->change_meassurement(Z_TORQUE_MEASSURE);
//		fmg->Move();
//		torque_ze = -fmg->get_meassurement()/weight;
//		std::cout<<"pomiar 4: "<<fmg->weight<<std::endl;
//		std::cout<<"t3: "<<fmg->weight[5]/weight<<std::endl;
		for(int i = 0 ; i < T3_FOR ; i++)
		{
			fmg->Move();
			std::cout<<"pomiar 4: "<<fmg->weight<<std::endl;
			std::cout<<"t3: "<<fmg->weight[5]/weight<<std::endl;
			sleep(1);
		}
		//cout<<"weight: "<<weight<<endl<<"tx: "<<t_x<<endl<<"ty: "<<t_y<<endl<<"tz: "<<t_z<<endl; 
		
		ecp_termination_notice();
		ecp_wait_for_stop();
		break;
	}
	std::cout<<"end\n"<<std::endl;
};

// sprawdzic co robi ta metoda, gdzie, w jakich przypadkach jest uzywana
ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_tzu_cs_irp6ot(_config);
};


force_meassure_generator::force_meassure_generator(ecp_task& _ecp_task, int _what_to_meassure) :
	ecp_generator(_ecp_task)
{
	what_to_meassure = _what_to_meassure;
	for(int i  = 0 ; i < 6 ; i++)
		weight[i] = 0;
}

bool force_meassure_generator::first_step()
{
	the_robot->EDP_data.instruction_type = GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.get_arm_type = FRAME;
	the_robot->EDP_data.next_interpolation_type
			= TCIM;

	return true;
}

Ft_v_vector *force_meassure_generator::get_meassurement()
{
	return &weight;
}

void force_meassure_generator::change_meassurement(int what)
{
	what_to_meassure = what;
}

bool force_meassure_generator::next_step()
{
	// transformacja ciezaru do osi z ukladu bazowego
	Homog_matrix current_frame_wo_offset(the_robot->EDP_data.current_arm_frame);
	current_frame_wo_offset.remove_translation();

	//	std::cout <<"frame: " <<	current_frame_wo_offset << std::endl;

	Ft_v_vector force_torque(the_robot->EDP_data.current_force_xyz_torque_xyz);

	weight = force_torque;
//	for(int i = 0 ; i < T3_FOR ; i++)
//	{
//		std::cout<<"w generatorze: "<<weight<<std::endl;
//		sleep(1);
//	}
	// weight = -force_torque[what_to_meassure];
	// for(int i  = 0 ; i < 6 ; i++)
	//	weight[i] = -force_torque[i];
	
	return false;
}

