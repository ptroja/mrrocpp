// -------------------------------------------------------------------------
//
// MP Master Process - methods for task sporadic coordination
//
// -------------------------------------------------------------------------

#include <cstdio>
#include <unistd.h>
#include <cstring>
#include <iostream>

#include <list>
#include <map>

#include <boost/foreach.hpp>

#include <libxml/xmlmemory.h>
#include <libxml/tree.h>
#include <libxml/debugXML.h>
#include <libxml/parser.h>
#include <libxml/xinclude.h>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "base/ecp_mp/ecp_mp_sensor.h"

#include "base/mp/mp_task.h"

#include "mp_t_fsautomat.h"

#include "ecp_mp_tr_rc_windows.h"
#include "State.h"
#include "StateHeap.h"

#include "base/lib/datastr.h"
#include "generator/ecp/newsmooth/ecp_mp_g_newsmooth.h"
#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "cube_face.h"
#include "CubeState.h"

#include "robot/conveyor/mp_r_conveyor.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"

#include "robot/bird_hand/mp_r_bird_hand.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"

#include "robot/sarkofag/mp_r_sarkofag.h"
#include "robot/festival/const_festival.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new fsautomat(_config);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void fsautomat::create_robots()
{


#if (R_BIRD_HAND == 1)
	ACTIVATE_MP_ROBOT(bird_hand);
#endif


#if (R_012 == 1)
	ACTIVATE_MP_ROBOT(conveyor);
	ACTIVATE_MP_ROBOT(irp6ot_tfg);
	ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_tfg);
	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(sarkofag);
#endif


	ACTIVATE_MP_DEFAULT_ROBOT(festival);

}

fsautomat::fsautomat(lib::configurator &_config) :
		task(_config)
{/*

 int size;
 char *filePath;
 std::string fileName;
 fileName = config.value<std::string>("xml_file", "[xml_settings]");
 xmlNode *cur_node, *child_node;
 xmlChar *stateType;
 size = 1 + mrrocpp_network_path.length() + fileName.length();
 filePath = new char[size];

 sprintf(filePath, "%s%s", mrrocpp_network_path.c_str(), fileName.c_str());
 // open xml document
 xmlDocPtr doc;
 doc = xmlParseFile(filePath);
 if(doc == NULL)
 {
 printf("ERROR in ecp initialization: could not parse file: %s\n",fileName.c_str());
 return;
 }
 // XML root
 xmlNode *root = NULL;
 root = xmlDocGetRootElement(doc);
 if(!root || !root->name)
 {
 printf("ecp initialization ERROR: Bad root node name!");
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
 if ( child_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(child_node->name, (const xmlChar *)"base/mp") )
 {
 for(;child_node->children; child_node->children = child_node->children->next)
 {
 if(child_node->children->type == XML_ELEMENT_NODE && !xmlStrcmp(child_node->children->name, (const xmlChar *)"cube_state"))
 {
 //argument = xmlNodeGetContent(child_node->children);
 //if(argument && xmlStrcmp(argument, (const xmlChar *)""));
 cube_state = new common::CubeState();
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
 //*/

	if (config.value <int>("vis_servoing")) {

	}

	// Konfiguracja wszystkich czujnikow
	BOOST_FOREACH(ecp_mp::sensor_item_t & sensor_item, sensor_m)
			{
				sensor_item.second->configure_sensor();
			}

	/*// dodanie transmitter'a
	 transmitter_m[ecp_mp::transmitter::TRANSMITTER_RC_WINDOWS]
	 = new ecp_mp::transmitter::rc_windows(ecp_mp::transmitter::TRANSMITTER_RC_WINDOWS, "[transmitter_rc_windows]", *this);
	 */
}

common::State fsautomat::createState(xmlNodePtr stateNode)
{
	common::State actState;

	xmlChar * stateID = xmlGetProp(stateNode, (const xmlChar *) "id");

	if (stateID) {
		actState.setStateID((char*) stateID);
	}

	xmlChar * stateType = xmlGetProp(stateNode, (const xmlChar *) "type");

	if (stateType) {
		actState.setType((char*) stateType);
	}
	// For each child of state: i.e. Robot
	for (xmlNodePtr child_node = stateNode->children; child_node != NULL; child_node = child_node->next) {
		if (child_node->type == XML_ELEMENT_NODE) {
			if (!xmlStrcmp(child_node->name, (const xmlChar *) "ECPGeneratorType")) {

				xmlChar * ecpGeneratorType = xmlNodeGetContent(child_node);
				if (ecpGeneratorType)
					actState.setGeneratorType((const char *) ecpGeneratorType);
				xmlFree(ecpGeneratorType);
			} else if (!xmlStrcmp(child_node->name, (const xmlChar *) "ROBOT")) {

				xmlChar * robot = xmlNodeGetContent(child_node);
				if (robot)
					actState.setRobot((char*) robot);
				xmlFree(robot);
			} else if (!xmlStrcmp(child_node->name, (const xmlChar *) "SetOfRobots")) {
				actState.robotSet = common::State::RobotSets();
				for (xmlNodePtr cchild_node = child_node->children; cchild_node != NULL;
						cchild_node = cchild_node->next) {
					if (cchild_node->type == XML_ELEMENT_NODE
							&& !xmlStrcmp(cchild_node->name, (const xmlChar *) "FirstSet")) {
						//actState.robotSet->firstSetCount = ((xmlLsCountNode(cchild_node)) - 1) / 2;
						//actState.robotSet->firstSet = new lib::robot_name_t[actState.robotSet->firstSetCount];
						for (xmlNodePtr set_node = cchild_node->children; set_node != NULL; set_node = set_node->next)
							if (set_node->type == XML_ELEMENT_NODE
									&& !xmlStrcmp(set_node->name, (const xmlChar *) "ROBOT"))
								actState.robotSet->firstSet.push_back(lib::returnProperRobot((char *) xmlNodeGetContent(set_node)));
					}
					/*if (cchild_node->type == XML_ELEMENT_NODE
					 && !xmlStrcmp(cchild_node->name, (const xmlChar *) "SecSet")) {
					 //actState.robotSet->secondSetCount = ((xmlLsCountNode(cchild_node)) - 1) / 2;
					 //actState.robotSet->secondSet = new lib::robot_name_t[actState.robotSet->secondSetCount];
					 for (xmlNodePtr set_node = cchild_node->children; set_node != NULL; set_node = set_node->next)
					 if (set_node->type == XML_ELEMENT_NODE
					 && !xmlStrcmp(set_node->name, (const xmlChar *) "ROBOT"))
					 actState.robotSet->secondSet.push_back(lib::returnProperRobot((char *) xmlNodeGetContent(set_node)));
					 }*/
				}
			} else if (!xmlStrcmp(child_node->name, (const xmlChar *) "TrajectoryFilePath")
					|| !xmlStrcmp(child_node->name, (const xmlChar *) "GeneratorParameters")
					|| !xmlStrcmp(child_node->name, (const xmlChar *) "Parameters")
					|| !xmlStrcmp(child_node->name, (const xmlChar *) "Sensor")
					|| !xmlStrcmp(child_node->name, (const xmlChar *) "Speech")) {
				xmlChar * stringArgument = xmlNodeGetContent(child_node);
				//askubis
				if (stringArgument) {
					std::cout << "ARGUMENT STRINGOWY:                            " << (char *) stringArgument
							<< std::endl;
					actState.setStringArgument((std::string) (char*) stringArgument);
				}
				xmlFree(stringArgument);
			} else if (!xmlStrcmp(child_node->name, (const xmlChar *) "TimeSpan")
					|| !xmlStrcmp(child_node->name, (const xmlChar *) "AddArg")) {
				xmlChar * numArgument = xmlNodeGetContent(child_node);
				if (numArgument)
					actState.setNumArgument((const char *) numArgument);
				xmlFree(numArgument);
			} else if (!xmlStrcmp(child_node->name, (const xmlChar *) "transition")) {
				//printf("name: %s\n", (char *)child_node->name);
				xmlChar *cond = xmlGetProp(child_node, (const xmlChar *) "condition");
				xmlChar *trans = xmlGetProp(child_node, (const xmlChar *) "target");
				if (cond && trans)
					actState.setTransition((const char *) cond, (const char *) trans, config);
			}
		}
	}
	xmlFree(stateType);
	xmlFree(stateID);
	return actState;
}

fsautomat::stateMap_t fsautomat::takeStatesMap()
{
	stateMap_t statesMap;

	std::string fileName(config.value <std::string>("xml_file", "[xml_settings]"));
	std::string filePath("../");
	filePath += fileName;

	std::cout << "XML FilePath: " << filePath << std::endl;

	// open xml document
	xmlDocPtr doc = xmlParseFile(filePath.c_str());
	xmlXIncludeProcess(doc);
	if (doc == NULL) {
		std::cout << "ERROR: could not parse file: \"" << fileName << "\"." << std::endl;
		return statesMap;
	}

	// XML root
	xmlNode *root = xmlDocGetRootElement(doc);
	if (!root || !root->name) {
		std::cout << "Bad root node name!" << std::endl;
		xmlFreeDoc(doc);
		return statesMap;
	}

	// for each root children
	for (xmlNodePtr cur_node = root->children; cur_node != NULL; cur_node = cur_node->next) {
		if (cur_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cur_node->name, (const xmlChar *) "SubTask")) {
			for (xmlNodePtr child_node = cur_node->children; child_node != NULL; child_node = child_node->next) {
				if (child_node->type == XML_ELEMENT_NODE && !xmlStrcmp(child_node->name, (const xmlChar *) "State")) {
					common::State actState = createState(child_node);
					statesMap.insert(stateMap_t::value_type(actState.getStateID(), actState));
				}
			}
		}
		if (cur_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cur_node->name, (const xmlChar *) "State")) {
			common::State actState = createState(cur_node);
			statesMap.insert(stateMap_t::value_type(actState.getStateID(), actState));
		}

	}
	// free the document
	xmlFreeDoc(doc);
	// free the global variables that may
	// have been allocated by the parser
	xmlCleanupParser();

	return statesMap;
}

void fsautomat::configureProperSensor(const char *propSensor)
{
	// Powolanie czujnikow

	if (config.value <int>("vis_servoing")) {

	}

	// Konfiguracja wszystkich czujnikow
	BOOST_FOREACH(ecp_mp::sensor_item_t & sensor_item, sensor_m)
			{
				sensor_item.second->configure_sensor();
			}
}

void fsautomat::configureProperTransmitter(const char *propTrans)
{
	// dodanie transmitter'a
	transmitter_m[ecp_mp::transmitter::TRANSMITTER_RC_WINDOWS] =
			new ecp_mp::transmitter::rc_windows(ecp_mp::transmitter::TRANSMITTER_RC_WINDOWS, "[transmitter_rc_windows]", *this);
}

void fsautomat::stopProperGen(const common::State &state)
{
	if (!state.robotSet)
		send_end_motion_to_ecps(state.getRobot());
	// TODO
	//send_end_motion_to_ecps(state.robotSet->firstSet.size(), state.robotSet->firstSet);
}

void fsautomat::runWaitFunction(const common::State &state)
{
	wait_ms(state.getNumArgument());
}

void fsautomat::runEmptyGen(const common::State &state)
{
	//TODO
	//run_extended_empty_gen_base(state.getNumArgument(), 1, (state.getRobot()).c_str());
	std::vector <lib::robot_name_t> myvector;
	myvector.push_back(state.getRobot());
	wait_for_task_termination(true, myvector);
}

void fsautomat::runEmptyGenForSet(const common::State &state)
{
	//TODO
	//run_extended_empty_gen_and_wait(state.robotSet->firstSetCount, state.robotSet->secondSetCount, state.robotSet->firstSet, state.robotSet->secondSet);
	wait_for_task_termination(true, state.robotSet->firstSet);
}

void fsautomat::executeMotion(const common::State &state)
{
	std::cout << "STATE STRING w executeMotion:  " << state.getStringArgument() << std::endl;
	int trjConf = config.value <int>("trajectory_from_xml", "[xml_settings]");
	if (trjConf && state.getGeneratorType() == ecp_mp::generator::ECP_GEN_NEWSMOOTH) {
		//	set_next_ecp_state(state.getGeneratorType(), state.getNumArgument(), state.getStateID(), state.getRobot());
	} else {
		//	set_next_ecp_state(state.getGeneratorType(), state.getNumArgument(), state.getStringArgument().c_str(), state.getRobot());
	}
}

void fsautomat::sensorInitialization()
{
	/*
	 BOOST_FOREACH(ecp_mp::sensor_item_t & sensor_item, sensor_m) {
	 sensor_item.second->configure_sensor();
	 }
	 */
}

void fsautomat::initializeCubeState(common::State &state)
{
	common::CUBE_COLOR colors[6];
	char *colorStr = strdup(state.getStringArgument().c_str());
	//	printf("color config: %s\n", colorStr);
	char *temp;
	int index = 0;
	for (temp = strtok(colorStr, " \t"); temp != NULL; temp = strtok(NULL, " \t")) {
		if (!strcmp(temp, "BLUE"))
			colors[index++] = common::BLUE;
		else if (!strcmp(temp, "GREEN"))
			colors[index++] = common::GREEN;
		else if (!strcmp(temp, "RED"))
			colors[index++] = common::RED;
		else if (!strcmp(temp, "ORANGE"))
			colors[index++] = common::ORANGE;
		else if (!strcmp(temp, "WHITE"))
			colors[index++] = common::WHITE;
		else if (!strcmp(temp, "YELLOW"))
			colors[index++] = common::YELLOW;
	}
	//	for(int i=0; i<6; i++)
	//		printf("c[%d]: %d\n", i, colors[i]);
	cube_state.set_state(colors[0], colors[1], colors[2], colors[3], colors[4], colors[5]);
}

void fsautomat::initiateSensorReading(common::State &state)
{
	/*        sensor_m[lib::SENSOR_CAMERA_ON_TRACK]->initiate_reading();
	 if (wait_ms(1000))
	 {
	 return true;
	 }
	 sensor_m[lib::SENSOR_CAMERA_ON_TRACK]->get_reading();
	 */
	/*	char *sensorName = strdup(state.getStringArgument());
	 SENSOR_t whichSensor;
	 if(!strcmp(sensorName, (const char *)"SENSOR_CAMERA_ON_TRACK"))
	 whichSensor = SENSOR_CAMERA_ON_TRACK;
	 else
	 whichSensor = SENSOR_CAMERA_ON_TRACK;

	 sensor_m[whichSensor]->initiate_reading();
	 */
}

void fsautomat::getSensorReading(common::State &state)
{
	/*	char *sensorName = strdup(state.getStringArgument());
	 SENSOR_t whichSensor;
	 if(!strcmp(sensorName, (const char *)"SENSOR_CAMERA_ON_TRACK"))
	 whichSensor = SENSOR_CAMERA_ON_TRACK;
	 else
	 whichSensor = SENSOR_CAMERA_ON_TRACK;

	 sensor_m[whichSensor]->get_reading();
	 */
}

void fsautomat::writeCubeState(common::State &state)
{
	int index = state.getNumArgument();

	ecp_mp::sensor::sensor <mrrocpp::mp::task::cube_face_t> * cube_recognition = dynamic_cast <ecp_mp::sensor::sensor <
			mrrocpp::mp::task::cube_face_t> *>(sensor_m[mrrocpp::ecp_mp::sensor::SENSOR_CAMERA_ON_TRACK]);

	cube_recognition->initiate_reading();
	wait_ms(1000);
	cube_recognition->get_reading();

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cube_state.cube_tab[index][3 * i + j] = (char) cube_recognition->image.colors[3 * i + j];

	printf("\nFACE FACE %d:\n", index);
	for (int i = 0; i < 9; i++) {
		switch (cube_state.cube_tab[index][i])
		{
			case 1:
				cube_state.cube_tab[index][i] = 'r';
				printf("R");
				break;
			case 2:
				cube_state.cube_tab[index][i] = 'o';
				printf("O");
				break;
			case 3:
				cube_state.cube_tab[index][i] = 'y';
				printf("Y");
				break;
			case 4:
				cube_state.cube_tab[index][i] = 'g';
				printf("G");
				break;
			case 5:
				cube_state.cube_tab[index][i] = 'b';
				printf("B");
				break;
			case 6:
				cube_state.cube_tab[index][i] = 'w';
				printf("W");
				break;
			default:
				cube_state.cube_tab[index][i] = 'o';
				printf("?");
				break;
		}
	}
	printf("\n");

}

void fsautomat::changeCubeState(common::State &state)
{
	int turn_angle = state.getNumArgument();
	common::CubeState tmp_cube_state;

	tmp_cube_state.set_state(cube_state, turn_angle);

	cube_state = tmp_cube_state;
}

void fsautomat::changeCubeState(int turn_angle)
{
	common::CubeState tmp_cube_state;

	tmp_cube_state.set_state(cube_state, turn_angle);

	cube_state = tmp_cube_state;
}

void fsautomat::communicate_with_windows_solver(common::State &state)
{
	//		  state.setProperTransitionResult(true);
	//		  return false;

	char c_up, c_right, c_front, c_down, c_left, c_back;
	int s, str_size;
	char cube_tab_send[55];
	char manipulation_sequence[200];

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cube_tab_send[2 * 9 + 3 * i + j] = cube_state.cube_tab[0][3 * i + j]; //rot cl 0

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cube_tab_send[1 * 9 + 3 * j + 2 - i] = cube_state.cube_tab[1][3 * i + j]; //rot cl 90

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cube_tab_send[3 * 9 + 3 * (2 - j) + i] = cube_state.cube_tab[2][3 * i + j]; //rot ccl 90

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cube_tab_send[5 * 9 + 3 * i + j] = cube_state.cube_tab[3][3 * i + j]; //rot cl 0

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cube_tab_send[4 * 9 + 3 * j + 2 - i] = cube_state.cube_tab[4][3 * i + j]; //rot cl 90

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cube_tab_send[0 * 9 + 3 * j + 2 - i] = cube_state.cube_tab[5][3 * i + j]; //rot cl 90

	printf("SEQ IN COLOR : %s\n", cube_tab_send);

	c_up = cube_tab_send[4];
	c_right = cube_tab_send[13];
	c_front = cube_tab_send[22];
	c_down = cube_tab_send[31];
	c_left = cube_tab_send[40];
	c_back = cube_tab_send[49];

	printf("%c %c %c %c %c %c\n", c_up, c_right, c_front, c_down, c_left, c_back);

	for (int i = 0; i < 54; i++) {
		if (cube_tab_send[i] == c_up)
			cube_tab_send[i] = 'u';
		else if (cube_tab_send[i] == c_down)
			cube_tab_send[i] = 'd';
		else if (cube_tab_send[i] == c_front)
			cube_tab_send[i] = 'f';
		else if (cube_tab_send[i] == c_back)
			cube_tab_send[i] = 'b';
		else if (cube_tab_send[i] == c_right)
			cube_tab_send[i] = 'r';
		else if (cube_tab_send[i] == c_left)
			cube_tab_send[i] = 'l';
	}

	cube_tab_send[54] = '\0';

	printf("SEQ FROM VIS : %s\n", cube_tab_send);

	//reszta

	// czyszczenie listy
	manipulation_list.clear();

	ecp_mp::transmitter::transmitter_base * transmitter_ptr = transmitter_m[ecp_mp::transmitter::TRANSMITTER_RC_WINDOWS];
	assert(transmitter_ptr);

	ecp_mp::transmitter::rc_windows * rc_solver_ptr = dynamic_cast <ecp_mp::transmitter::rc_windows *>(transmitter_ptr);
	assert(rc_solver_ptr);

	ecp_mp::transmitter::rc_windows & rc_solver = *rc_solver_ptr;

	for (int i = 0; i < 54; i++) {
		rc_solver.to_va.rc_state[i] = cube_tab_send[i];
	}
	//mp_object.transmitter_m[TRANSMITTER_RC_WINDOWS]->to_va.rc_windows.rc_state[i]=patternx[i];
	rc_solver.to_va.rc_state[54] = '\0';

	rc_solver.t_write();

	rc_solver.t_read(true);

	printf("OPS: %s", rc_solver.from_va.sequence);

	strcpy(manipulation_sequence, rc_solver.from_va.sequence);

	if ((manipulation_sequence[0] == 'C') && (manipulation_sequence[1] == 'u') && (manipulation_sequence[2] == 'b')
			&& (manipulation_sequence[3] == 'e')) {
		printf("Jam jest daltonista. ktory Ci nie uloz*y kostki\n");
		//manipulation_sequence_computed = false;
		state.setProperTransitionResult(false);
		return;
	}

	//sekwencja poczatkowa w kolejnosci: UP, DOWN, FRONT, BACK, LEFT, RIGHT
	//cube_initial_state=BGROWY
	s = 0;
	str_size = 0;
	for (unsigned int char_i = 0; char_i < strlen(rc_solver.from_va.sequence) - 1; char_i++) {
		if (s == 0) {
			switch (rc_solver.from_va.sequence[char_i])
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
			s = 1;
			str_size++;
		} else if (s == 1) {
			switch (rc_solver.from_va.sequence[char_i])
			{
				case ' ':
					manipulation_sequence[str_size] = '1';
					s = 0;
					break;
				case '2':
					manipulation_sequence[str_size] = '2';
					s = 2;
					break;
				case '\'':
					manipulation_sequence[str_size] = '3';
					s = 2;
					break;
			}
			str_size++;
		} else if (s == 2) {
			s = 0;
		}

	}

	if (s == 1) {
		str_size--;
		manipulation_sequence[str_size] = '1';
		str_size++;
	}
	manipulation_sequence[str_size] = '\0';

	printf("\n%d %zd\n", str_size, strlen(manipulation_sequence));
	printf("SEQ from win %s\n", rc_solver.from_va.sequence);
	printf("\nSEQ2 %s\n", manipulation_sequence);

	//pocztaek ukladania
	// dodawanie manipulacji do listy
	for (unsigned int char_i = 0; char_i < strlen(manipulation_sequence) - 1; char_i += 2) {
		manipulation_list.push_back(common::SingleManipulation(common::read_cube_color(manipulation_sequence[char_i]), common::read_cube_turn_angle(manipulation_sequence[char_i
				+ 1])));
	}
	//manipulation_sequence_computed = true;
	state.setProperTransitionResult(true);
}

void fsautomat::translateManipulationSequence(common::StateHeap &sh)
{
	std::list <const char *> scenario;

	BOOST_FOREACH(const common::SingleManipulation & manipulation, manipulation_list)
			{
				if (manipulation.face_to_turn == cube_state.getUp()) {
					scenario.push_back("fco_CL_90_1");
					changeCubeState(90);
				} else if (manipulation.face_to_turn == cube_state.getDown()) {
					scenario.push_back("fco_CCL_90_1");
					changeCubeState(270);
				} else if (manipulation.face_to_turn == cube_state.getFront()) {
					scenario.push_back("fco_CL_0_1");
					changeCubeState(0);
					scenario.push_back("fto_CL_0_1");
					scenario.push_back("fco_CL_90_1");
					changeCubeState(90);
				} else if (manipulation.face_to_turn == cube_state.getRear()) {
					scenario.push_back("fco_CL_0_1");
					changeCubeState(0);
					scenario.push_back("fto_CL_0_1");
					scenario.push_back("fco_CCL_90_1");
					changeCubeState(270);
				} else if (manipulation.face_to_turn == cube_state.getLeft()) {
					scenario.push_back("fco_CL_0_1");
					changeCubeState(0);
				} else if (manipulation.face_to_turn == cube_state.getRight()) {
					scenario.push_back("fco_CL_180_1");
					changeCubeState(180);
				}
				switch (manipulation.turn_angle)
				{
					case common::CL_90:
						scenario.push_back("fto_CL_90_1");
						break;
					case common::CL_0:
						scenario.push_back("fto_CL_0_1");
						break;
					case common::CCL_90:
						scenario.push_back("fto_CCL_90_1");
						break;
					case common::CL_180:
						scenario.push_back("fto_CL_180_1");
						break;
					default:
						break;
				}
			}

	scenario.reverse();
	BOOST_FOREACH(const char * ptr, scenario)
			{
				sh.pushTargetName(ptr);
			}
	sh.showHeapContent();
}

void fsautomat::main_task_algorithm(void)
{
	common::StateHeap sh;

	break_state = false;

	stateMap_t stateMap = takeStatesMap();
	std::cout << "Mapa zawiera: " << stateMap.size() << std::endl;
	//	std::cout<<"ELEMENTOW INIT jest: "<<stateMap->count((const char *)"INIT")<<std::endl;

	sr_ecp_msg->message("Nowa seria");

	// adding first state name
	//strcmp(nextState, (char *)"INIT");

	std::string nextState = "INIT";

	// temporary sensor config in this place
	BOOST_FOREACH(ecp_mp::sensor_item_t & s, sensor_m)
			{
				s.second->configure_sensor();
			}

	for (; nextState != "_STOP_"; nextState = stateMap[nextState].returnNextStateID(sh)) {
		if (nextState == "_END_") {
			nextState = sh.popTargetName();
		}

		// protection from wrong targetID specyfication
		if (stateMap.count(nextState) == 0) {
			//std::cout<<"ASKUBIS error, state not found: "<<nextState<<std::endl;
			break;
		}

		const std::string & currentStateType = stateMap[nextState].getType();

		if (currentStateType == "set_next_ecp_state") {
			executeMotion(stateMap[nextState]);
			std::cout << nextState << " -> zakonczony" << std::endl;

		} else if (currentStateType == "wait_for_task_termination") {
			runEmptyGenForSet(stateMap[nextState]);
			std::cout << nextState << " -> zakonczony" << std::endl;

		} else if (currentStateType == "emptyGen") {
			runEmptyGen(stateMap[nextState]);
			std::cout << nextState << " -> zakonczony" << std::endl;

		} else if (currentStateType == "wait_ms") {
			runWaitFunction(stateMap[nextState]);
			std::cout << nextState << " -> zakonczony" << std::endl;

		} else if (currentStateType == "send_end_motion_to_ecps") {
			stopProperGen(stateMap[nextState]);
			std::cout << nextState << " -> zakonczony" << std::endl;

		} else if (currentStateType == "systemInitialization") {
			std::cout << "In system initialization.." << std::endl;
			sensorInitialization();
			std::cout << nextState << " -> zakonczony" << std::endl;

		} else if (currentStateType == "cubeStateInit") {
			initializeCubeState(stateMap[nextState]);
			std::cout << nextState << " -> zakonczony" << std::endl;

		} else if (currentStateType == "initiateSensorReading") {
			initiateSensorReading(stateMap[nextState]);
			std::cout << nextState << " -> zakonczony" << std::endl;

		} else if (currentStateType == "getSensorReading") {
			getSensorReading(stateMap[nextState]);
			std::cout << nextState << " -> zakonczony" << std::endl;

		} else if (currentStateType == "cubeStateWriting") {
			writeCubeState(stateMap[nextState]);
			std::cout << nextState << " -> zakonczony" << std::endl;

		} else if (currentStateType == "cubeStateChange") {
			changeCubeState(stateMap[nextState]);
			std::cout << nextState << " -> zakonczony" << std::endl;

		} else if (currentStateType == "communicateWithSolver") {
			communicate_with_windows_solver(stateMap[nextState]);
			std::cout << nextState << " -> zakonczony" << std::endl;

		} else if (currentStateType == "manipulationSeqTranslation") {
			translateManipulationSequence(sh);
			std::cout << nextState << " -> zakonczony" << std::endl;
		}
	}
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
