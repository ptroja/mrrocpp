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
};

/** destruktor **/
ecp_task_tzu_postument_test::~ecp_task_tzu_postument_test()
{
};


// methods for ECP template to redefine in concrete classes
void ecp_task_tzu_postument_test::task_initialization(void) 
{
	// ecp_m_robot = new ecp_irp6_on_track_robot (*this);
	if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
	{
		ecp_m_robot = new ecp_irp6_on_track_robot (*this);
	}
	else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
	{
		ecp_m_robot = new ecp_irp6_postument_robot (*this);
	}
	befg = new bias_edp_force_generator(*this);
	fmg = new force_meassure_generator(*this);	
	// fmg->set_configuration(1,5);
	ftcg = new ecp_force_tool_change_generator(*this);
	tcg = new ecp_tool_change_generator(*this,true);
	ynrfg = new ecp_tff_nose_run_generator(*this,8);
	sr_ecp_msg->message("ECP loaded");
};

void ecp_task_tzu_postument_test::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP cs irp6ot  - pushj start in tzu");
	ecp_wait_for_start();
	
	while(true)			
	{
		cout<<"START"<<endl;
		befg->Move();
		cout<<"Biasowanie dokonane"<<endl;
		tcg->set_tool_parameters(0,0,0.09); 
		tcg->Move();
		cout<<"Puszczenie nose generatora"<<endl;
		sleep(1);
		ynrfg->Move();
		ecp_termination_notice();
		ecp_wait_for_stop();
		break;
	}
	std::cout<<"end\n"<<std::endl;
};

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
