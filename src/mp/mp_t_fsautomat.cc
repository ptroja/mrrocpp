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
#include "libxml/debugXML.h"

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
#include "mp/StateHeap.h"

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
/*	int size, conArg;
	char *filePath;
	char *fileName = config.return_string_value("xml_file", "[xml_settings]");
   xmlNode *cur_node, *child_node;
   xmlChar *stateType, *argument;

	size = 1 + strlen(mrrocpp_network_path) + strlen(fileName);				  	
	filePath = new char[size];

	sprintf(filePath, "%s%s", mrrocpp_network_path, fileName);
   // open xml document
   xmlDocPtr doc;
   doc = xmlParseFile(filePath);
   if(doc == NULL)
	{
      printf("ERROR in ecp initialization: could not parse file: %s\n",fileName);
		return;
	}

   // XML root
   xmlNode *root = NULL;
   root = xmlDocGetRootElement(doc);
   if(!root || !root->name)
   {
      printf("ECP initialization ERROR: Bad root node name!");
      xmlFreeDoc(doc);
      return;
   }

   // for each root children "state"
   for(cur_node = root->children; cur_node != NULL; cur_node = cur_node->next)
   {
      if ( cur_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(cur_node->name, (const xmlChar *) "State" ) )
      {
         stateType = xmlGetProp(cur_node, (const xmlChar *) "type");
			if(!xmlStrcmp(stateType, (const xmlChar *)"systemInitialization"))
			{
				while(xmlStrcmp(cur_node->children->name, (const xmlChar *)"taskInit"))
					cur_node->children = cur_node->children->next;
	         // For each child of state: i.e. Robot
	         for(child_node = cur_node->children->children; child_node != NULL; child_node = child_node->next)
	         {
	            if ( child_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(child_node->name, (const xmlChar *)"mp") )
	            {
						for(;child_node->children; child_node->children = child_node->children->next)
						{
							if(child_node->children->type == XML_ELEMENT_NODE && 
									!xmlStrcmp(child_node->children->name, (const xmlChar *)"cube_state"))
							{									
								//argument = xmlNodeGetContent(child_node->children);
								//if(argument && xmlStrcmp(argument, (const xmlChar *)""));
									cube_state = new CubeState();
								//xmlFree(argument);
							}
							if(child_node->children->type == XML_ELEMENT_NODE && 
									!xmlStrcmp(child_node->children->name, (const xmlChar *)"Sensor"))
							{
								//argument = xmlNodeGetContent(child_node->children);
								//if(argument && xmlStrcmp(argument, (const xmlChar *)""))
									configureProperSensor((char *)"");
								//xmlFree(argument);
							}
							if(child_node->children->type == XML_ELEMENT_NODE && 
									!xmlStrcmp(child_node->children->name, (const xmlChar *)"Transmitter"))
							{
								//argument = xmlNodeGetContent(child_node->children);									
								//if(argument && xmlStrcmp(argument, (const xmlChar *)""))
									configureProperTransmitter((char *) "");
								//xmlFree(argument);
							}
						}
	            }
	         }
			}
         xmlFree(stateType);
		}
   }
   xmlFreeDoc(doc);
   xmlCleanupParser();
	*/
    sensor_m[SENSOR_CAMERA_ON_TRACK] =
        new ecp_mp_vis_sensor (SENSOR_CAMERA_ON_TRACK, "[vsp_vis_eih]", *this);

    if (config.return_int_value("vis_servoing"))
    {

    sensor_m[SENSOR_CAMERA_SA] =
        new ecp_mp_vis_sensor (SENSOR_CAMERA_SA, "[vsp_vis_sac]", *this);
        
        }

    // Konfiguracja wszystkich czujnikow
    for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
            sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
    {
        sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
        sensor_m_iterator->second->configure_sensor();
    }

    usleep(1000*100);

    // dodanie transmitter'a
    transmitter_m[TRANSMITTER_RC_WINDOWS] =
        new rc_windows_transmitter (TRANSMITTER_RC_WINDOWS, "[transmitter_rc_windows]", *this);
	
	 cube_state = new CubeState();

	sr_ecp_msg->message("MP fsautomat loaded");
};

std::map<char *, State, ecp_task::str_cmp> * mp_task_fsautomat::takeStatesMap()
{
	int size, index;
	char *filePath;
	char *fileName = config.return_string_value("xml_file", "[xml_settings]");
   xmlNode *cur_node, *child_node, *cchild_node, *set_node;
   xmlChar *ecpGeneratorType, *robot, *robotSet;
   xmlChar *stateID, *stateType, *stringArgument, *numArgument;
	xmlChar *cond, *trans;
	std::map<char *, State, ecp_task::str_cmp> * statesMap = new std::map<char *, State, ecp_task::str_cmp>();

	size = 1 + strlen(mrrocpp_network_path) + strlen(fileName);				  	
	filePath = new char[size];

	sprintf(filePath, "%s%s", mrrocpp_network_path, fileName);
   // open xml document
   xmlDocPtr doc;
   doc = xmlParseFile(filePath);
   if(doc == NULL)
	{
      cout<<"ERROR: could not parse file: \""<<fileName<<"\"."<<endl;
		return statesMap;
	}

   // XML root
   xmlNode *root = NULL;
   root = xmlDocGetRootElement(doc);
   if(!root || !root->name)
   {
      cout<<"Bad root node name!"<<endl;
      xmlFreeDoc(doc);
      return statesMap;
   }
	State *actState = NULL;

   // for each root children "state"
   for(cur_node = root->children; cur_node != NULL; cur_node = cur_node->next)
   {
      if ( cur_node->type == XML_ELEMENT_NODE  &&
            !xmlStrcmp(cur_node->name, (const xmlChar *) "State" ) )
      {
			actState = new State();
         stateID = xmlGetProp(cur_node, (const xmlChar *) "id");
         if(stateID)
			{
				actState->setStateID((char*)stateID);
			}
         stateType = xmlGetProp(cur_node, (const xmlChar *) "type");
         if(stateType)
         {
				actState->setType((char*)stateType);
         }
         // For each child of state: i.e. Robot
         for(child_node = cur_node->children; child_node != NULL; child_node = child_node->next)
         {
            if ( child_node->type == XML_ELEMENT_NODE  &&
                  !xmlStrcmp(child_node->name, (const xmlChar *)"ECPGeneratorType") )
            {
               ecpGeneratorType = xmlNodeGetContent(child_node);
               if(ecpGeneratorType)
						actState->setGeneratorType((char*)ecpGeneratorType);
               xmlFree(ecpGeneratorType);
            }
            if ( child_node->type == XML_ELEMENT_NODE  &&
                  !xmlStrcmp(child_node->name, (const xmlChar *)"ROBOT") )
            {
               robot = xmlNodeGetContent(child_node);
               if(robot)
						actState->setRobot((char*)robot);
               xmlFree(robot);
            }
            if ( child_node->type == XML_ELEMENT_NODE  &&
                  !xmlStrcmp(child_node->name, (const xmlChar *)"SetOfRobots") )
            {
					actState->robotSet = new State::RobotSets();
					for(cchild_node = child_node->children; cchild_node != NULL; cchild_node = cchild_node->next)
					{
		            if ( cchild_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(cchild_node->name, (const xmlChar *)"FirstSet") )
						{
							actState->robotSet->firstSetCount = ((xmlLsCountNode(cchild_node))-1)/2;
							actState->robotSet->firstSet = new ROBOT_ENUM[actState->robotSet->firstSetCount];
							index = 0;
							for(set_node = cchild_node->children; set_node!=NULL; set_node = set_node->next)
								if ( set_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(set_node->name, (const xmlChar *)"ROBOT") )
									actState->robotSet->firstSet[index++] = State::returnProperRobot((char *)xmlNodeGetContent(set_node));
						}
		            if ( cchild_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(cchild_node->name, (const xmlChar *)"SecSet") )
						{
							actState->robotSet->secondSetCount = ((xmlLsCountNode(cchild_node))-1)/2;
							actState->robotSet->secondSet = new ROBOT_ENUM[actState->robotSet->secondSetCount];
							index = 0;
							for(set_node = cchild_node->children; set_node!=NULL; set_node = set_node->next)
								if ( set_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(set_node->name, (const xmlChar *)"ROBOT") )
									actState->robotSet->secondSet[index++] = State::returnProperRobot((char *)xmlNodeGetContent(set_node));
						}
					}
            }
            if ( child_node->type == XML_ELEMENT_NODE  &&
                  (!xmlStrcmp(child_node->name, (const xmlChar *)"TrajectoryFilePath") ||
						 !xmlStrcmp(child_node->name, (const xmlChar *)"GeneratorParameters") ||
						 !xmlStrcmp(child_node->name, (const xmlChar *)"Parameters") ||
						 !xmlStrcmp(child_node->name, (const xmlChar *)"Sensor") ||
						 !xmlStrcmp(child_node->name, (const xmlChar *)"Speech")))
            {
               stringArgument = xmlNodeGetContent(child_node);
               if(stringArgument)
						actState->setStringArgument((char*)stringArgument);
               xmlFree(stringArgument);
            }
            if ( child_node->type == XML_ELEMENT_NODE  &&
                  (!xmlStrcmp(child_node->name, (const xmlChar *)"TimeSpan") ||
						 !xmlStrcmp(child_node->name, (const xmlChar *)"AddArg")))
            {
               numArgument = xmlNodeGetContent(child_node);
               if(numArgument)
						actState->setNumArgument((char*)numArgument);
               xmlFree(numArgument);
            }
            if ( child_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(child_node->name, (const xmlChar *)"transition") )
            {
					//printf("name: %s\n", (char *)child_node->name);
					cond = xmlGetProp(child_node, (const xmlChar *) "condition");
					trans = xmlGetProp(child_node, (const xmlChar *) "target");
					if(cond && trans)
						actState->setTransition((char *)cond, (char *)trans, config);
            }
         }
			statesMap->insert(std::map<char *, State>::value_type(actState->getStateID(), *actState));
         xmlFree(stateType);
         xmlFree(stateID);
		}
   }
   // free the document
   xmlFreeDoc(doc);
   // free the global variables that may
   // have been allocated by the parser
   xmlCleanupParser();
	return statesMap;
}

void mp_task_fsautomat::configureProperSensor(char *propSensor)
{
    // Powolanie czujnikow
    sensor_m[SENSOR_CAMERA_ON_TRACK] = new ecp_mp_vis_sensor (SENSOR_CAMERA_ON_TRACK, "[vsp_vis_eih]", *this);

    if (config.return_int_value("vis_servoing"))
	 {
		 sensor_m[SENSOR_CAMERA_SA] = new ecp_mp_vis_sensor (SENSOR_CAMERA_SA, "[vsp_vis_sac]", *this);
	 }

    // Konfiguracja wszystkich czujnikow
    for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
            sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
    {
        sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
        sensor_m_iterator->second->configure_sensor();
    }
    usleep(1000*100);
}

void mp_task_fsautomat::configureProperTransmitter(char *propTrans)
{
    // dodanie transmitter'a
    transmitter_m[TRANSMITTER_RC_WINDOWS] =
        new rc_windows_transmitter (TRANSMITTER_RC_WINDOWS, "[transmitter_rc_windows]", *this);
}

bool mp_task_fsautomat::stopProperGen(State &state)
{
	if(state.robotSet == NULL)
		if (send_end_motion_to_ecps (1, state.getRobot()))
		{	return true;	}
	if (send_end_motion_to_ecps (state.robotSet->firstSetCount,state.robotSet->firstSet))
	{	return true;	}
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
	if (run_ext_empty_gen(state.getNumArgument(), 1, state.getRobot())) 
	{
		return true; 
	}		
	return false;
}

bool mp_task_fsautomat::runEmptyGenForSet(State &state)
{
	if (run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
			(state.robotSet->firstSetCount, state.robotSet->secondSetCount, state.robotSet->firstSet, state.robotSet->secondSet))
	{	return true; }
	else
		return false;
}

bool mp_task_fsautomat::executeMotion(State &state)
{
	int trjConf = config.return_int_value("trajectory_from_xml", "[xml_settings]");
	if(trjConf && state.getGeneratorType() == ECP_GEN_SMOOTH)
	{
		if(set_next_ecps_state( (int) state.getGeneratorType(), state.getNumArgument(), state.getStateID(), 1, state.getRobot()))
		{	return true;	}
	}
	else
	{
		if(set_next_ecps_state( (int) state.getGeneratorType(), state.getNumArgument(), state.getStringArgument(), 1, state.getRobot()))
		{	return true;	}
	}
	
	return false;
}

bool mp_task_fsautomat::sensorInitialization()
{
/*	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
			sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}
	*/
	return false;
}

bool mp_task_fsautomat::initializeCubeState(State &state)
{
	CUBE_COLOR colors[6];
	char *colorStr = strdup(state.getStringArgument());
//	printf("color config: %s\n", colorStr);
	char *temp;
	int index =0;
	for(temp = strtok(colorStr, " \t"); temp != NULL; temp = strtok(NULL, " \t"))
	{
		if(!strcmp(temp, (const char *)"BLUE"))
			colors[index++] = BLUE;
		if(!strcmp(temp, (const char *)"GREEN"))
			colors[index++] = GREEN;
		if(!strcmp(temp, (const char *)"RED"))
			colors[index++] = RED;
		if(!strcmp(temp, (const char *)"ORANGE"))
			colors[index++] = ORANGE;
		if(!strcmp(temp, (const char *)"WHITE"))
			colors[index++] = WHITE;
		if(!strcmp(temp, (const char *)"YELLOW"))
			colors[index++] = YELLOW;
	}
//	for(int i=0; i<6; i++)
//		printf("c[%d]: %d\n", i, colors[i]);
	cube_state->set_state(colors[0], colors[1], colors[2], colors[3], colors[4], colors[5]);
	return false;
}
	
bool mp_task_fsautomat::initiateSensorReading(State &state)
{
/*        sensor_m[SENSOR_CAMERA_ON_TRACK]->initiate_reading();
        if (wait_ms(1000))
        {
           return true;
        }
        sensor_m[SENSOR_CAMERA_ON_TRACK]->get_reading();
*/		  
/*	char *sensorName = strdup(state.getStringArgument());
	SENSOR_ENUM whichSensor;		
	if(!strcmp(sensorName, (const char *)"SENSOR_CAMERA_ON_TRACK"))
		whichSensor = SENSOR_CAMERA_ON_TRACK;
	else
		whichSensor = SENSOR_CAMERA_ON_TRACK;
	
	sensor_m[whichSensor]->initiate_reading();
*/
	return false;
}

bool mp_task_fsautomat::getSensorReading(State &state)
{
/*	char *sensorName = strdup(state.getStringArgument());
	SENSOR_ENUM whichSensor;		
	if(!strcmp(sensorName, (const char *)"SENSOR_CAMERA_ON_TRACK"))
		whichSensor = SENSOR_CAMERA_ON_TRACK;
	else
		whichSensor = SENSOR_CAMERA_ON_TRACK;

	sensor_m[whichSensor]->get_reading();
*/
	return false;
}

bool mp_task_fsautomat::writeCubeState(State &state)
{
	int index = state.getNumArgument();
        
	sensor_m[SENSOR_CAMERA_ON_TRACK]->initiate_reading();
	if (wait_ms(1000))
	{
		return true;
	}
	sensor_m[SENSOR_CAMERA_ON_TRACK]->get_reading();
	
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			cube_state->cube_tab[index][3*i+j]=(char)sensor_m[SENSOR_CAMERA_ON_TRACK]->image.cube_face.colors[3*i+j];
	
	
	printf("\nFACE FACE %d:\n",index);
	for(int i=0; i<9; i++)
	{
		switch (cube_state->cube_tab[index][i])
		{
			case 1:
				cube_state->cube_tab[index][i]='r';
				printf("R");
				break;
			case 2:
				cube_state->cube_tab[index][i]='o';
				printf("O");                
				break;
			case 3:
				cube_state->cube_tab[index][i]='y';
				printf("Y");
				break;
			case 4:
				cube_state->cube_tab[index][i]='g';
				printf("G");
				break;
			case 5:
				cube_state->cube_tab[index][i]='b';
				printf("B");
				break;
			case 6:
				cube_state->cube_tab[index][i]='w';
				printf("W");
				break;
			default:
				cube_state->cube_tab[index][i]='o';
				printf("?");
				break;
		}
	}
	printf("\n");
	
	return false;
}

bool mp_task_fsautomat::changeCubeState(State &state)
{
	int turn_angle = state.getNumArgument();
	CubeState tmp_cube_state;
	
	tmp_cube_state.set_state(*cube_state, turn_angle);
	
	*cube_state = tmp_cube_state;
	return false;
}


bool mp_task_fsautomat::communicate_with_windows_solver(State &state)
{
//		  state.setProperTransitionResult(true);
//		  return false;
		  
    char c_up, c_right, c_front, c_down, c_left, c_back;
    int s, str_size;
    char cube_tab_send[54];
    char manipulation_sequence[200];

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            cube_tab_send[2*9+3*i+j]=cube_state->cube_tab[0][3*i+j]; //rot cl 0

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            cube_tab_send[1*9+3*j+2-i]=cube_state->cube_tab[1][3*i+j]; //rot cl 90

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            cube_tab_send[3*9+3*(2-j)+i]=cube_state->cube_tab[2][3*i+j]; //rot ccl 90

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            cube_tab_send[5*9+3*i+j]=cube_state->cube_tab[3][3*i+j]; //rot cl 0

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            cube_tab_send[4*9+3*j+2-i]=cube_state->cube_tab[4][3*i+j]; //rot cl 90

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            cube_tab_send[0*9+3*j+2-i]=cube_state->cube_tab[5][3*i+j]; //rot cl 90

    printf("SEQ IN COLOR : %s\n",cube_tab_send);

    c_up=cube_tab_send[4];
    c_right=cube_tab_send[13];
    c_front=cube_tab_send[22];
    c_down=cube_tab_send[31];
    c_left=cube_tab_send[40];
    c_back=cube_tab_send[49];

    printf("%c %c %c %c %c %c\n", c_up, c_right, c_front, c_down, c_left, c_back);

    for(int i=0; i<54; i++)
    {
        if (cube_tab_send[i] == c_up)
            cube_tab_send[i]='u';
        else if (cube_tab_send[i] == c_down)
            cube_tab_send[i]='d';
        else if (cube_tab_send[i] == c_front)
            cube_tab_send[i]='f';
        else if (cube_tab_send[i] == c_back)
            cube_tab_send[i]='b';
        else if (cube_tab_send[i] == c_right)
            cube_tab_send[i]='r';
        else if (cube_tab_send[i] == c_left)
            cube_tab_send[i]='l';
    }


    cube_tab_send[54]='\0';

    printf("SEQ FROM VIS : %s\n",cube_tab_send);

    //reszta
    // struktura pomiocnicza
    SingleManipulation single_manipulation;

    // czyszczenie listy
    manipulation_list.clear();

    for(int i=0; i<54; i++)
    {
        transmitter_m[TRANSMITTER_RC_WINDOWS]->to_va.rc_windows.rc_state[i]=cube_tab_send[i];
    }
    //mp_object.transmitter_m[TRANSMITTER_RC_WINDOWS]->to_va.rc_windows.rc_state[i]=patternx[i];
    transmitter_m[TRANSMITTER_RC_WINDOWS]->to_va.rc_windows.rc_state[54]='\0';


    transmitter_m[TRANSMITTER_RC_WINDOWS]->t_write();



    transmitter_m[TRANSMITTER_RC_WINDOWS]->t_read(true);

    printf ("OPS: %s", transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence);

    strcpy (manipulation_sequence,transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence);

    if ((manipulation_sequence[0]=='C') && (manipulation_sequence[1]=='u') && (manipulation_sequence[2]=='b') && (manipulation_sequence[3]=='e'))
    {
        printf("Jam jest daltonista. ktory Ci nie uloz*y kostki\n");
        //manipulation_sequence_computed = false;
		  state.setProperTransitionResult(false);
        return false;
    }

    //sekwencja poczatkowa w kolejnosci: UP, DOWN, FRONT, BACK, LEFT, RIGHT
    //cube_initial_state=BGROWY
    s=0;
    str_size=0;
    for (unsigned int char_i=0; char_i < strlen(transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence)-1; char_i ++)
    {
        if (s==0)
        {
            switch (transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence[char_i])
            {
            case 'U':
                manipulation_sequence[str_size] = 'B';
                break;
            case 'D':
                manipulation_sequence[str_size] = 'G';
                break;
            case 'F':
                manipulation_sequence[str_size] = 'O';
                break;
            case 'B':
                manipulation_sequence[str_size] = 'R';
                break;
            case 'L':
                manipulation_sequence[str_size] = 'W';
                break;
            case 'R':
                manipulation_sequence[str_size] = 'Y';
                break;
            }
            s=1;
            str_size++;
        }
        else if (s==1)
        {
            switch (transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence[char_i])
            {
            case ' ':
                manipulation_sequence[str_size] = '1';
                s=0;
                break;
            case '2':
                manipulation_sequence[str_size] = '2';
                s=2;
                break;
            case '\'':
                manipulation_sequence[str_size] = '3';
                s=2;
                break;
            }
            str_size++;
        }
        else if (s==2)
        {
            s=0;
        }

    }

    if (s==1)
    {
        str_size--;
        manipulation_sequence[str_size] = '1';
        str_size++;
    }
    manipulation_sequence[str_size]='\0';

    printf ("\n%d %d\n",str_size,strlen(manipulation_sequence));
    printf ("SEQ from win %s\n",transmitter_m[TRANSMITTER_RC_WINDOWS]->from_va.rc_windows.sequence);
    printf ("\nSEQ2 %s\n",manipulation_sequence);

    //pocztaek ukladania
    // dodawanie manipulacji do listy
    for (unsigned int char_i=0; char_i < strlen(manipulation_sequence)-1; char_i += 2)
    {
        single_manipulation.set_state(read_cube_color(manipulation_sequence[char_i]),
                                      read_cube_turn_angle(manipulation_sequence[char_i+1]));
        manipulation_list.push_back(single_manipulation);
    }
    //manipulation_sequence_computed = true;
	 state.setProperTransitionResult(true);

    return false;
}

bool mp_task_fsautomat::translateManipulationSequence(StateHeap &sh)
{
	manipulation_list.reverse();
	for(std::list<SingleManipulation>::iterator manipulation_list_iterator = manipulation_list.begin();
			manipulation_list_iterator != manipulation_list.end(); manipulation_list_iterator++)
	{
		switch(manipulation_list_iterator->turn_angle)
		{
			case CL_90:
				sh.pushTargetName("fto_CL_90_1");
				break;
			case CL_0:
				sh.pushTargetName("fto_CL_0_1");
				break;
			case CCL_90:
				sh.pushTargetName("fto_CCL_90_1");
				break;
			case CL_180:
				sh.pushTargetName("fto_CL_180_1");
				break;
			default:
				break;
		}
		if (manipulation_list_iterator->face_to_turn == cube_state->getUp())
		{	
			sh.pushTargetName("fco_CL_90_1");
		}
		else if (manipulation_list_iterator->face_to_turn == cube_state->getDown())
		{
			sh.pushTargetName("fco_CCL_90_1");
		}
		else if (manipulation_list_iterator->face_to_turn == cube_state->getFront())
		{
			sh.pushTargetName("fco_CL_90_1");
			sh.pushTargetName("fto_CL_0_1");
			sh.pushTargetName("fco_CL_0_1");
		}
		else if (manipulation_list_iterator->face_to_turn == cube_state->getRear())
		{
			sh.pushTargetName("fco_CCL_90_1");
			sh.pushTargetName("fto_CL_0_1");
			sh.pushTargetName("fco_CL_0_1");
		}
		else if (manipulation_list_iterator->face_to_turn == cube_state->getLeft())
		{
			sh.pushTargetName("fco_CL_0_1");
		}
		else if (manipulation_list_iterator->face_to_turn == cube_state->getRight())
		{
			sh.pushTargetName("fco_CL_180_1");
		}
	}
	// sh.pushTargetName("name");
	return false;
}

void mp_task_fsautomat::main_task_algorithm(void)
{
	StateHeap sh;
	break_state=false;
	char *nextState = new char[64];
//	std::list<State> *statesList = takeStatesList();
	std::map<char *, State, ecp_task::str_cmp> * stateMap = takeStatesMap();
	std::cout<<"Mapa zawiera: "<<stateMap->size()<<std::endl;
	std::cout<<"ELEMENTOW INIT jest: "<<stateMap->count((const char *)"INIT")<<std::endl;
	

	sr_ecp_msg->message("MP dla Automatu Skonczonego - wcisnij start");
	wait_for_start ();
	// Wyslanie START do wszystkich ECP 
	start_all (robot_m);

	for (;;)
	{  // Wewnetrzna petla
		sr_ecp_msg->message("Nowa seria");
		// adding first state name
		//strcmp(nextState, (char *)"INIT");
		sprintf(nextState, "INIT");
		// temporary sensor config in this place
	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
			sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	//std::cout<<"###### "<<(*stateMap)[nextState].getType()<<std::endl;
		for(;strcmp(nextState, (const char *)"STOP"); strcpy(nextState, (*stateMap)[nextState].returnNextStateID(sh)))
		{
			if(!strcmp(nextState, (const char *)"_END_"))
				strcpy(nextState, sh.popTargetName());
			// protection from wrong targetID specyfication
			if(stateMap->count(nextState)==0)
				break;
			if(strcmp((*stateMap)[nextState].getType(), (const char *)"runGenerator") == 0)
			{
				if(!executeMotion((*stateMap)[nextState]))
					std::cout<<nextState<<" -> zakonczony"<<std::endl;
				//continue;
			}
			if(strcmp((*stateMap)[nextState].getType(), (const char *)"emptyGenForSet") == 0)
			{
				if(!runEmptyGenForSet((*stateMap)[nextState]))
					std::cout<<nextState<<" -> zakonczony"<<std::endl;
				//continue;
			}
			if(strcmp((*stateMap)[nextState].getType(), (const char *)"emptyGen") == 0)
			{
				if(!runEmptyGen((*stateMap)[nextState]))
					std::cout<<nextState<<" -> zakonczony"<<std::endl;
				//continue;
			}
			if(strcmp((*stateMap)[nextState].getType(), (const char *)"wait") == 0)
			{
				if(!runWaitFunction((*stateMap)[nextState]))
					std::cout<<nextState<<" -> zakonczony"<<std::endl;
				//continue;
			}
			if(strcmp((*stateMap)[nextState].getType(), (const char *)"stopGen") == 0)
			{
				if(!stopProperGen((*stateMap)[nextState]))
					std::cout<<nextState<<" -> zakonczony"<<std::endl;
				//continue;
			}
			if(strcmp((*stateMap)[nextState].getType(), (const char *)"systemInitialization") == 0)
			{
				std::cout<<"In sensor initialization.."<<std::endl;
				if(!sensorInitialization())
					std::cout<<nextState<<" -> zakonczony"<<std::endl;
				//continue;
			}
			if(strcmp((*stateMap)[nextState].getType(), (const char *)"cubeStateInit") == 0)
			{
				if(!initializeCubeState((*stateMap)[nextState]))
					std::cout<<nextState<<" -> zakonczony"<<std::endl;
				//continue;
			}
			if(strcmp((*stateMap)[nextState].getType(), (const char *)"initiateSensorReading") == 0)
			{
				if(!initiateSensorReading((*stateMap)[nextState]))
					std::cout<<nextState<<" -> zakonczony"<<std::endl;
				//continue;
			}
			if(strcmp((*stateMap)[nextState].getType(), (const char *)"getSensorReading") == 0)
			{
				if(!getSensorReading((*stateMap)[nextState]))
					std::cout<<nextState<<" -> zakonczony"<<std::endl;
				//continue;
			}
			if(strcmp((*stateMap)[nextState].getType(), (const char *)"cubeStateWriting") == 0)
			{
				if(!writeCubeState((*stateMap)[nextState]))
					std::cout<<nextState<<" -> zakonczony"<<std::endl;
				//continue;
			}
			if(strcmp((*stateMap)[nextState].getType(), (const char *)"cubeStateChange") == 0)
			{
				if(!changeCubeState((*stateMap)[nextState]))
					std::cout<<nextState<<" -> zakonczony"<<std::endl;
				//continue;
			}
			if(strcmp((*stateMap)[nextState].getType(), (const char *)"communicateWithSolver") == 0)
			{
				if(!communicate_with_windows_solver((*stateMap)[nextState]))
					std::cout<<nextState<<" -> zakonczony"<<std::endl;
				//continue;
			}
			if(strcmp((*stateMap)[nextState].getType(), (const char *)"manipulationSeqTranslation") == 0)
			{
				if(!translateManipulationSequence(sh))
					std::cout<<nextState<<" -> zakonczony"<<std::endl;
				//continue;
			}
		}
		// Oczekiwanie na STOP od UI
		wait_for_stop (MP_THROW);
  
   	// Wyslanie STOP do wszystkich ECP po zakonczeniu programu uzytkownika
		terminate_all (robot_m);
		break; 

	} // koniec: for(;;) - wewnetrzna petla
}
