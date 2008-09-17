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

bool ecp_task_fsautomat_irp6ot::str_cmp::operator()(char const *a, char const *b) const
{
	return strcmp(a, b)<0;
}

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


bool ecp_task_fsautomat_irp6ot::loadTrajectories()
{
	int size;
	char * filePath;
	Trajectory* actTrajectory;
	xmlNode *cur_node, *child_node, *cchild_node, *ccchild_node;
	xmlChar *coordinateType, *numOfPoses, *robot;	 
	xmlChar *xmlDataLine;
	xmlChar *stateName, *stateType;
	
	xmlDocPtr doc;
	size = 1 + strlen(mrrocpp_network_path) + strlen("data/automat_ver1.xml");				  	
	filePath = new char[size];

	// Stworzenie sciezki do pliku.
	strcpy(filePath, mrrocpp_network_path);
	sprintf(filePath, "%s%s", mrrocpp_network_path, "data/automat_ver1.xml");
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
	
	trjMap = new std::map<char*, Trajectory, ecp_task_fsautomat_irp6ot::str_cmp>();
	
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
							!xmlStrcmp(robot, (const xmlChar *)"ROBOT_IRP6_ON_TRACK"))
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
}

void ecp_task_fsautomat_irp6ot::main_task_algorithm(void)
{

	int size;				  	
	char * path1;

	loadTrajectories();
//	for(std::map<char *, Trajectory, ecp_task_fsautomat_irp6ot::str_cmp>::iterator ii = trjMap->begin(); ii != trjMap->end(); ++ii)
//	{
//		printf("Key: #%s#\n", (*ii).first);
//		(*ii).second.showTime();
//	}
	
	printf("Lista ON_TRACK zawiera: %d elementow\n", trjMap->size());
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
			
			switch ( (POURING_ECP_STATES) mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state)
			{
				case ECP_GEN_SMOOTH:
				  	size = 1 + strlen(mrrocpp_network_path) + strlen("data/automat_ver1.xml");				  	
					path1 = new char[size];
					// Stworzenie sciezki do pliku.
					strcpy(path1, mrrocpp_network_path);
					sprintf(path1, "%s%s", mrrocpp_network_path, "data/automat_ver1.xml");
					//sg->load_trajectory_from_xml(path1, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					sg->load_trajectory_from_xml((*trjMap)[mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string]);

					///---------------------------------------------------------------------
/*				  	size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);				  	
					path1 = new char[size];
					// Stworzenie sciezki do pliku.
					strcpy(path1, mrrocpp_network_path);
					sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.mp_package.ecp_next_state.mp_2_ecp_next_state_string);
					sg->load_file_with_path (path1);
*/
					//printf("\nON_TRACK ECP_GEN_SMOOTH :%s\n\n", path1);
					delete[] path1;
					//printf("OT po delete\n");
					sg->Move();
					//printf("OT po move\n");
					break;
				case ECP_GEN_POURING:				
					tcg->set_tool_parameters(-0.18, 0.0, 0.25);
					tcg->Move();
					break;
				case ECP_END_POURING:
					tcg->set_tool_parameters(0.0, 0.0, 0.25);
					tcg->Move();
					break;
				case GRIP:
					go_st->configure(-0.03, 1000);
					go_st->execute();
					break;
				case LET_GO:
					go_st->configure(0.03, 1000);
					go_st->execute();
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
