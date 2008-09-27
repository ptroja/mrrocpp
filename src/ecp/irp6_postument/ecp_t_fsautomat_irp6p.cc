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
#include "ecp/irp6_postument/ecp_t_fsautomat_irp6p.h"

#include "mp/Trajectory.h"

// KONSTRUKTORY
ecp_task_fsautomat_irp6p::ecp_task_fsautomat_irp6p(configurator &_config) : ecp_task(_config)
{
	sg = NULL;
};

ecp_task_fsautomat_irp6p::~ecp_task_fsautomat_irp6p(){};

/*bool ecp_task_fsautomat_irp6p::str_cmp::operator()(char const *a, char const *b) const
{
	return strcmp(a, b)<0;
}*/

void ecp_task_fsautomat_irp6p::task_initialization(void) 
{
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

/*bool ecp_task_fsautomat_irp6p::loadTrajectories(char * fileName)
{
	int size;
	char * filePath;
	Trajectory* actTrajectory;
	xmlNode *cur_node, *child_node, *cchild_node, *ccchild_node;
	xmlChar *coordinateType, *numOfPoses, *robot;	 
	xmlChar *xmlDataLine;
	xmlChar *stateName, *stateType;
	
	xmlDocPtr doc;
	size = 1 + strlen(mrrocpp_network_path) + strlen(fileName);				  	
	filePath = new char[size];

	// Stworzenie sciezki do pliku.
	//strcpy(filePath, mrrocpp_network_path);
	sprintf(filePath, "%s%s", mrrocpp_network_path, fileName);
	//printf("Plik: %s\n", filePath1);
	doc = xmlParseFile(filePath);
	if(doc == NULL)
	{
		throw ecp_generator::ECP_error(NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}
	
	xmlNode *root = NULL;
	root = xmlDocGetRootElement(doc);
	if(!root || !root->name)
	{
		xmlFreeDoc(doc);
		throw ecp_generator::ECP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
	}
	
	trjMap = new std::map<char*, Trajectory, ecp_task_fsautomat_irp6p::str_cmp>();
	
   for(cur_node = root->children; cur_node != NULL; cur_node = cur_node->next)
   {
      if ( cur_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(cur_node->name, (const xmlChar *) "State" ) )
      {
         stateName = xmlGetProp(cur_node, (const xmlChar *) "name");
			stateType = xmlGetProp(cur_node, (const xmlChar *) "type");
         if(stateName && !strcmp((const char *)stateType, (const char *)"motionExecute"))
			{
	         for(child_node = cur_node->children; child_node != NULL; child_node = child_node->next)
	         {
	            if ( cur_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(child_node->name, (const xmlChar *)"ROBOT") )
					{
						robot = xmlNodeGetContent(child_node);
					}
	            if ( cur_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(child_node->name, (const xmlChar *)"Trajectory") &&
							!xmlStrcmp(robot, (const xmlChar *)"ROBOT_IRP6_POSTUMENT"))
	            {
						coordinateType = xmlGetProp(child_node, (const xmlChar *)"coordinateType");
						numOfPoses = xmlGetProp(child_node, (const xmlChar *)"numOfPoses");
						actTrajectory = new Trajectory((char *)numOfPoses, (char *)stateName, (char *)coordinateType);
						for(cchild_node = child_node->children; cchild_node!=NULL; cchild_node = cchild_node->next)
						{							
	            		if ( cur_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(cchild_node->name, (const xmlChar *)"Pose") )
							{
								actTrajectory->createNewPose();
								for(ccchild_node = cchild_node->children; ccchild_node!=NULL; ccchild_node = ccchild_node->next)
								{
	            				if ( cur_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(ccchild_node->name, (const xmlChar *)"StartVelocity") )
									{	
										xmlDataLine = xmlNodeGetContent(ccchild_node);
										actTrajectory->setStartVelocities((char *)xmlDataLine);
										xmlFree(xmlDataLine);
									}
	            				if ( cur_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(ccchild_node->name, (const xmlChar *)"EndVelocity") )
									{										
										xmlDataLine = xmlNodeGetContent(ccchild_node);
										actTrajectory->setEndVelocities((char *)xmlDataLine);
										xmlFree(xmlDataLine);
									}
	            				if ( cur_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(ccchild_node->name, (const xmlChar *)"Velocity") )
									{										
										xmlDataLine = xmlNodeGetContent(ccchild_node);
										actTrajectory->setVelocities((char *)xmlDataLine);
										xmlFree(xmlDataLine);
									}
	            				if ( cur_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(ccchild_node->name, (const xmlChar *)"Accelerations") )
									{										
										xmlDataLine = xmlNodeGetContent(ccchild_node);
										actTrajectory->setAccelerations((char *)xmlDataLine);
										xmlFree(xmlDataLine);
									}
	            				if ( cur_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(ccchild_node->name, (const xmlChar *)"Coordinates") )
									{										
										xmlDataLine = xmlNodeGetContent(ccchild_node);
										actTrajectory->setCoordinates((char *)xmlDataLine);
										xmlFree(xmlDataLine);
									}
								}
								actTrajectory->addPoseToTrajectory();
							}
							
						}
						xmlFree(coordinateType);
						xmlFree(numOfPoses);
						xmlFree(stateType);
			         xmlFree(stateName);
						trjMap->insert(std::map<char *, Trajectory>::value_type(actTrajectory->getName(), *actTrajectory));
	            }
	         }
				xmlFree(robot);				
			}
		}
	}
	xmlFreeDoc(doc);
	xmlCleanupParser();
//	for(std::map<char *, Trajectory>::iterator ii = trjMap->begin(); ii != trjMap->end(); ++ii)
//		(*ii).second.showTime();
			
	return true;
}*/

void ecp_task_fsautomat_irp6p::main_task_algorithm(void)
{

	int size;				  	
	char * path1;
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
					sg->Move();
					break;
				case RCSC_GRIPPER_OPENING:
					double go_args[2];
					Trajectory::setValuesInArray(go_args, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					go_st->configure(go_args[0], (int)go_args[1]);
					go_st->execute();
					//delete[] args
					break;
/*				case WEIGHT:
					printf("force0: %d\n", sensor_m.begin()->second->image.force.rez[0]);
					printf("force1: %d\n", sensor_m.begin()->second->image.force.rez[0]);
					printf("force2: %d\n", sensor_m.begin()->second->image.force.rez[0]);
					break;*/
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
