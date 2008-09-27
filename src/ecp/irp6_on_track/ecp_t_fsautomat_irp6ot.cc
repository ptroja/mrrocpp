// ------------------------------------------------------------------------
//   ecp_t_fsautomat_irp6ot.cc - automat skonczony , ECP dla IRP6_ON_TRACK
// 
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
// 
// Ostatnia modyfikacja: 2008
// ------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#include "libxml/xmlmemory.h"
#include "libxml/parser.h"
#include "libxml/tree.h"

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "mp/Trajectory.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_fsautomat.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_t_fsautomat.h"
#include "ecp/irp6_on_track/ecp_t_fsautomat_irp6ot.h"

// KONSTRUKTORY
ecp_task_fsautomat_irp6ot::ecp_task_fsautomat_irp6ot(configurator &_config) : ecp_task(_config)
{
	sg = NULL;
	tcg = NULL;
};

ecp_task_fsautomat_irp6ot::~ecp_task_fsautomat_irp6ot()
{
	//EMPTY
};

// methods for ECP template to redefine in concrete classes
void ecp_task_fsautomat_irp6ot::task_initialization(void) 
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	 ecp_m_robot = new ecp_irp6_on_track_robot (*this); 
	
	sg = new ecp_smooth_generator (*this, true);
	tcg = new ecp_tool_change_generator(*this, true);

	go_st = new ecp_sub_task_gripper_opening(*this);
		
	sr_ecp_msg->message("ECP loaded");
};



void ecp_task_fsautomat_irp6ot::main_task_algorithm(void)
{

	int size;				  	
	char * path1;
	char * fileName = config.return_string_value("xml_file", "[xml_settings]");
	int trjConf = config.return_int_value("trajectory_from_xml", "[xml_settings]");
	int ecpLevel = config.return_int_value("trajectory_on_ecp_level", "[xml_settings]");

	if(trjConf && ecpLevel)
	{
		trjMap = loadTrajectories(fileName, ROBOT_IRP6_ON_TRACK);
		printf("Lista ROBOT_IRP6_ON_TRACK zawiera: %d elementow\n", trjMap->size());
	}
//	for(std::map<char *, Trajectory, ecp_task_fsautomat_irp6ot::str_cmp>::iterator ii = trjMap->begin(); ii != trjMap->end(); ++ii)
//	{
//		printf("Key: #%s#\n", (*ii).first);
//		(*ii).second.showTime();
//	}
	
//	printf("ontrack z konfiguracji: %s\n", config.return_string_value("node_name", "[mp]"));
//	printf("Elementow approach_1: %d\n", trjMap->count("depart_1"));
//	(*trjMap)["approach_1"].showTime();	

	sr_ecp_msg->message("ECP fsautomat irp6ot  - wcisnij start");
	ecp_wait_for_start();
	for(;;) { // Wewnetrzna petla nieskonczona
		
		for(;;) 
		{
			sr_ecp_msg->message("Waiting for MP order");

			get_next_state (); 
			
			sr_ecp_msg->message("Order received");
			
			switch ( (STATE_MACHINE_ECP_STATES) mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state)
			{
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
							// Stworzenie sciezki do pliku.
							//strcpy(path1, mrrocpp_network_path);
							sprintf(path1, "%s%s", mrrocpp_network_path, fileName);
							sg->load_trajectory_from_xml(path1, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
							delete[] path1;
						}
					}
					else
					{
					  	size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);				  	
						path1 = new char[size];
						// Stworzenie sciezki do pliku.
						//strcpy(path1, mrrocpp_network_path);
						sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
						sg->load_file_with_path (path1);
						delete[] path1;
					}
					//printf("\nON_TRACK ECP_GEN_SMOOTH :%s\n\n", path1);
					//printf("OT po delete\n");
					sg->Move();
					//printf("OT po move\n");
					break;
				case ECP_TOOL_CHANGE_GENERATOR:
					double tcg_args[3];
					Trajectory::setValuesInArray(tcg_args, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					tcg->set_tool_parameters(tcg_args[0], tcg_args[1], tcg_args[2]);
					tcg->Move();
					//delete[] tcg_args;
					break;
				case RCSC_GRIPPER_OPENING:
					double go_args[2];
					Trajectory::setValuesInArray(go_args, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					go_st->configure(go_args[0], (int)go_args[1]);
					go_st->execute();
					//delete[] go_args;
					break;
				default:
				break;
			}
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
	return new ecp_task_fsautomat_irp6ot(_config);
};
