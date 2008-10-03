// ------------------------------------------------------------------------
//   ecp_t_fsautomat_irp6p.cc - zadanie przelewania, ECP dla IRP6_POSTUMENT
// 
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
// 
// Ostatnia modyfikacja: 2008
// ------------------------------------------------------------------------


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <map>

#include "libxml/xmlmemory.h"
#include "libxml/parser.h"
#include "libxml/tree.h"

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_fsautomat.h"
#include "ecp_mp/ecp_mp_s_schunk.h"	

#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_t_fsautomat.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp/irp6_postument/ecp_t_fsautomat_irp6p.h"

#include "mp/Trajectory.h"

// KONSTRUKTORY
ecp_task_fsautomat_irp6p::ecp_task_fsautomat_irp6p(configurator &_config) : ecp_task(_config)
{
	sg = NULL;
	tcg = NULL;
	gt = NULL;
	nrg = NULL;
	rgg = NULL;
	gag = NULL;
	rfrg = NULL;
	tig = NULL;
	befg = NULL;
	wmg = NULL;
	
	go_st = NULL;
};

ecp_task_fsautomat_irp6p::~ecp_task_fsautomat_irp6p(){};

void ecp_task_fsautomat_irp6p::task_initialization(void) 
{
	// TODO: initialize elements depending on initialization section in xml
	 ecp_m_robot = new ecp_irp6_postument_robot (*this); 

	// Powolanie czujnikow
//	sensor_m[SENSOR_FORCE_POSTUMENT] = 
//		new ecp_mp_schunk_sensor (SENSOR_FORCE_POSTUMENT, "[vsp_force_irp6p]", *this);
				
/*	// Konfiguracja wszystkich czujnikow	
	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	usleep(1000*100);
*/
	sg = new ecp_smooth_generator (*this, true);

	go_st = new ecp_sub_task_gripper_opening(*this);
	
	sr_ecp_msg->message("ECP loaded");
};

void ecp_task_fsautomat_irp6p::main_task_algorithm(void)
{

	int size;				  	
	char * path1;
	double *gen_args;
	char * fileName = config.return_string_value("xml_file", "[xml_settings]");
	int trjConf = config.return_int_value("trajectory_from_xml", "[xml_settings]");
	int ecpLevel = config.return_int_value("trajectory_on_ecp_level", "[xml_settings]");

	if(trjConf && ecpLevel)
	{
		trjMap = loadTrajectories(fileName, ROBOT_IRP6_POSTUMENT);
		printf("Lista ROBOT_IRP6_POSTUMENT zawiera: %d elementow\n", trjMap->size());
	}
	
	sr_ecp_msg->message("ECP fsautomat irp6p  - wcisnij start");
	ecp_wait_for_start();
	for(;;) { // Wewnetrzna petla nieskonczona
		
		for(;;) {
			sr_ecp_msg->message("Waiting for MP order");

			get_next_state (); 
			
			sr_ecp_msg->message("Order received");
			
			switch ( (STATE_MACHINE_ECP_STATES) mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state)
			{
				case ECP_GEN_TEACH_IN:
					size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					path1 = new char[size];
					sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					tig->flush_pose_list();
					tig->load_file_with_path (path1);
					tig->initiate_pose_list();
					delete[] path1;
					tig->Move();
					break;
				case ECP_GEN_SMOOTH:
					if(trjConf)
					{
						if(ecpLevel)
						{
							sg->load_trajectory_from_xml((*trjMap)[mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string]);
						}
						else
						{
				  			size = 1 + strlen(mrrocpp_network_path) + strlen(fileName);
							path1 = new char[size];
							sprintf(path1, "%s%s", mrrocpp_network_path, fileName);
							sg->load_trajectory_from_xml(path1, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
							delete[] path1;
						}
					}
					else
					{
					  	size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);				  	
						path1 = new char[size];
						sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
						sg->load_file_with_path (path1);
						delete[] path1;
					}
					sg->Move();
					break;
            case ECP_WEIGHT_MEASURE_GENERATOR:
					wmg->Move();
					break;
				case ECP_GEN_TRANSPARENT:
					gt->Move();
					break;
				case ECP_GEN_BIAS_EDP_FORCE:
					befg->Move();
					break;
				case ECP_GEN_TFF_NOSE_RUN:
					nrg->Move();
					break;
				case ECP_GEN_TFF_RUBIK_GRAB:
					gen_args = new double[4];
					size = Trajectory::setValuesInArray(gen_args, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					if(size > 3)
						rgg->configure(gen_args[0], gen_args[1], (int)gen_args[2], (bool)gen_args[3]);
					else	
						rgg->configure(gen_args[0], gen_args[1], (int)gen_args[2]);
					rgg->Move();
					delete[] gen_args;
					break;
            case ECP_GEN_TFF_RUBIK_FACE_ROTATE:
					gen_args = new double[1];
					Trajectory::setValuesInArray(gen_args, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					rfrg->configure(gen_args[0]);
					rfrg->Move();
					delete[] gen_args;
					break;
            case ECP_GEN_TFF_GRIPPER_APPROACH:
					gen_args = new double[2];
					Trajectory::setValuesInArray(gen_args, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					gag->configure(gen_args[0] , (int)gen_args[1]);
					gag->Move();
					delete[] gen_args;
					break;
				case ECP_TOOL_CHANGE_GENERATOR:
					gen_args = new double[3];
					Trajectory::setValuesInArray(gen_args, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					tcg->set_tool_parameters(gen_args[0], gen_args[1], gen_args[2]);
					tcg->Move();
					delete[] gen_args;
					break;
				case RCSC_GRIPPER_OPENING:
					gen_args = new double[2];
					Trajectory::setValuesInArray(gen_args, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					go_st->configure(gen_args[0], (int)gen_args[1]);
					go_st->execute();
					delete[] gen_args;
					break;
				default:
				break;
			} // end switch
			ecp_termination_notice();
			
		} //end for
		
		// Oczekiwanie na STOP
		printf("przed wait for stop\n");
		ecp_wait_for_stop ();
		break;
	} // koniec: for(;;) wewnetrznej
	
};

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_fsautomat_irp6p(_config);
};
