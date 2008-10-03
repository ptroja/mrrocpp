// -------------------------------------------------------------------------
// 
// MP Master Process - methods for task sporadic coordination
// 
// -------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <iostream.h>

#include "libxml/xmlmemory.h"
#include "libxml/parser.h"
#include "libxml/tree.h"

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_t_fsautomat.h"

#include <list>
#include <map>

#include "mp/mp_g_force.h"
#include "mp/mp_g_vis.h"
#include "ecp_mp/ecp_mp_s_vis.h"
#include "ecp_mp/ecp_mp_tr_rc_windows.h"
#include "mp/State.h"

mp_task_fsautomat::mp_task_fsautomat(configurator &_config) : mp_task(_config)
{

}

mp_task_fsautomat::~mp_task_fsautomat()
{
 
}	

mp_task* return_created_mp_task (configurator &_config)
{
	return new mp_task_fsautomat(_config);
}



// methods fo mp template to redefine in concete class
void mp_task_fsautomat::task_initialization(void) 
{
/*	// Powolanie czujnikow
	sensor_m[SENSOR_FORCE_ON_TRACK]=new ecp_mp_schunk_sensor(SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);
		
	sensor_m[SENSOR_FORCE_POSTUMENT] = new ecp_mp_schunk_sensor(SENSOR_FORCE_POSTUMENT, "[vsp_force_irp6p]", *this);
		
	// Konfiguracja wszystkich czujnikow	
	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}
			
	usleep(1000*100);
*/
	sr_ecp_msg->message("MP fsautomat loaded");
};

std::list<State> * mp_task_fsautomat::takeStatesList()
{
	int size;
	char *filePath;
	char *fileName = config.return_string_value("xml_file", "[xml_settings]");
   xmlNode *cur_node, *child_node;
   xmlChar *ecpGeneratorType, *robot;
   xmlChar *stateName, *stateType, *stringArgument, *numArgument;
	std::list<State> *statesList = new std::list<State>();

	size = 1 + strlen(mrrocpp_network_path) + strlen(fileName);				  	
	filePath = new char[size];

	sprintf(filePath, "%s%s", mrrocpp_network_path, fileName);
   // open xml document
   xmlDocPtr doc;
   doc = xmlParseFile(filePath);
   if(doc == NULL)
	{
      cout<<"ERROR: could not parse file: \""<<fileName<<"\"."<<endl;
		return statesList;
	}

   // XML root
   xmlNode *root = NULL;
   root = xmlDocGetRootElement(doc);
   if(!root || !root->name)
   {
      cout<<"Bad root node name!"<<endl;
      xmlFreeDoc(doc);
      return statesList;
   }
	State *actState = NULL;

   // for each root children "state"
   for(cur_node = root->children; cur_node != NULL; cur_node = cur_node->next)
   {
		actState = new State();
      if ( cur_node->type == XML_ELEMENT_NODE  &&
            !xmlStrcmp(cur_node->name, (const xmlChar *) "State" ) )
      {
         stateName = xmlGetProp(cur_node, (const xmlChar *) "name");
         if(stateName)
			{
				actState->setName((char*)stateName);
			}
         stateType = xmlGetProp(cur_node, (const xmlChar *) "type");
         if(stateType)
         {
				actState->setType((char*)stateType);
         }
         // For each child of state: i.e. Robot
         for(child_node = cur_node->children; child_node != NULL; child_node = child_node->next)
         {
            if ( cur_node->type == XML_ELEMENT_NODE  &&
                  !xmlStrcmp(child_node->name, (const xmlChar *)"ECPGeneratorType") )
            {
               ecpGeneratorType = xmlNodeGetContent(child_node);
               if(ecpGeneratorType)
						actState->setGeneratorType((char*)ecpGeneratorType);
               xmlFree(ecpGeneratorType);
            }
            if ( cur_node->type == XML_ELEMENT_NODE  &&
                  !xmlStrcmp(child_node->name, (const xmlChar *)"ROBOT") )
            {
               robot = xmlNodeGetContent(child_node);
               if(robot)
						actState->setRobot((char*)robot);
               xmlFree(robot);
            }
            if ( cur_node->type == XML_ELEMENT_NODE  &&
                  (!xmlStrcmp(child_node->name, (const xmlChar *)"TrajectoryFilePath") ||
						 !xmlStrcmp(child_node->name, (const xmlChar *)"GeneratorCoordinates") ||
						 !xmlStrcmp(child_node->name, (const xmlChar *)"Speech")))
            {
               stringArgument = xmlNodeGetContent(child_node);
               if(stringArgument)
						actState->setStringArgument((char*)stringArgument);
               xmlFree(stringArgument);
            }
            if ( cur_node->type == XML_ELEMENT_NODE  &&
                  (!xmlStrcmp(child_node->name, (const xmlChar *)"TimeSpan") ||
						 !xmlStrcmp(child_node->name, (const xmlChar *)"AddArg")))
            {
               stringArgument = xmlNodeGetContent(child_node);
               if(numArgument)
						actState->setNumArgument((char*)numArgument);
               xmlFree(numArgument);
            }
         }
			statesList->push_back(*actState);
			
         xmlFree(stateType);
         xmlFree(stateName);
		}
   }
   // free the document
   xmlFreeDoc(doc);
   // free the global variables that may
   // have been allocated by the parser
   xmlCleanupParser();
	return statesList;
}

bool mp_task_fsautomat::stopProperGen(State &state)
{
	if (send_end_motion_to_ecps (1, state.getRobot()))
	{
		return true;
	}
	return false;
}

bool mp_task_fsautomat::runWaitFunction(State &state)
{
	if(wait_ms(state.getNumArgument()))
	{
		return true;
	}
	return false;
}

bool mp_task_fsautomat::runEmptyGen(State &state)
{
	if (run_ext_empty_gen(false, 1, state.getRobot())) 
	{
		return true; 
	}		
	return false;
}

bool mp_task_fsautomat::runEmptyGenForSet(State &state)
{
//	std::cout<<"wchdze... "<<state.getName()<<std::endl;
	if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
			(2, 2, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT, ROBOT_IRP6_ON_TRACK, ROBOT_IRP6_POSTUMENT)) 
	{
		return true;
	}
	else
		return false;
}

bool mp_task_fsautomat::executeMotion(State &state)
{
	int trjConf = config.return_int_value("trajectory_from_xml", "[xml_settings]");
	if(trjConf && state.getGeneratorType() == ECP_GEN_SMOOTH)
	{
		if(set_next_ecps_state( (int) state.getGeneratorType(), state.getNumArgument(), state.getName(), 1, state.getRobot()))
		{	return true;	}
	}
	else
	{
		if(set_next_ecps_state( (int) state.getGeneratorType(), state.getNumArgument(), state.getStringArgument(), 1, state.getRobot()))
		{	return true;	}
	}
	
	return false;
}


void mp_task_fsautomat::main_task_algorithm(void)
{
	break_state=false;
	std::list<State> *statesList = takeStatesList();
	std::cout<<"Lista zawiera: "<<statesList->size()<<std::endl;
	std::list<State>::iterator i;
//	std::cout<<"Z konfiguracji: "<<config.return_int_value("trajectory_from_xml", "[xml_settings]")<<std::endl;
	
	
      // Oczekiwanie na zlecenie START od UI  
	sr_ecp_msg->message("MP dla Automatu Skonczonego - wcisnij start");
	wait_for_start ();
	// Wyslanie START do wszystkich ECP 
	start_all (robot_m);

	for (;;)
	{  // Wewnetrzna petla
	
		for(i=statesList->begin(); i!=statesList->end(); ++i)
		{
			//(*i).showStateContent();
			//continue;
			//cout<<"###### "<<(*i).getType()<<endl;
			if(strcmp((*i).getType(), (const char *)"runGenerator") == 0)
			{
				if(!executeMotion((*i)))
					std::cout<<(*i).getName()<<" -> zakonczony"<<std::endl;
				continue;
			}
			if(strcmp((*i).getType(), (const char *)"emptyGenForSet") == 0)
			{
				if(!runEmptyGenForSet((*i)))
					std::cout<<(*i).getName()<<" -> zakonczony"<<std::endl;
				//else
				//	std::cout<<"Blad!! zwrocono false! "<<(*i).getName()<<std::endl;
				continue;
			}
			if(strcmp((*i).getType(), (const char *)"emptyGen") == 0)
			{
				if(!runEmptyGen((*i)))
					std::cout<<(*i).getName()<<" -> zakonczony"<<std::endl;
				continue;
			}
			if(strcmp((*i).getType(), (const char *)"wait") == 0)
			{
				if(!runWaitFunction((*i)))
					std::cout<<(*i).getName()<<" -> zakonczony"<<std::endl;
				continue;
			}
			if(strcmp((*i).getType(), (const char *)"stopGen") == 0)
			{
				if(!stopProperGen((*i)))
					std::cout<<(*i).getName()<<" -> zakonczony"<<std::endl;
				continue;
			}
			
		}
		
        	// Oczekiwanie na STOP od UI
		wait_for_stop (MP_THROW);
  
   	// Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
		terminate_all (robot_m);
		break; 

	} // koniec: for(;;) - wewnetrzna petla
}
