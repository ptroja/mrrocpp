#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <stdio.h>

#include "lib/mis_fun.h"
#include "ecp/common/ecp_task.h"
#include "ecp/common/ECP_main_error.h"
#include "ecp/common/ecp_teach_in_generator.h"

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xinclude.h>

ecp_task::ecp_task(configurator &_config) :
	base(_config)
{
	sensor_m.clear();
	ecp_m_robot = NULL;
}

ecp_task::~ecp_task()
{
	delete[] mrrocpp_network_path;
}

bool ecp_task::pulse_check()
{
	struct sigevent stop_event;

	_pulse_msg ui_msg; // wiadomosc z ui

	stop_event.sigev_notify = SIGEV_UNBLOCK;// by Y zamiast creceive
	TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE, &stop_event, NULL, NULL); // czekamy na odbior pulsu stopu
	int rcvid = MsgReceive(trigger_attach->chid, &ui_msg, sizeof(ui_msg), NULL);

	if (rcvid == -1) {/* Error condition, exit */
		// perror("blad receive w reader\n");
	}

	if (rcvid == 0) {/* Pulse received */
		switch (ui_msg.hdr.code) {
			case _PULSE_CODE_DISCONNECT:
				/*
				 * A client disconnected all its connections (called
				 * name_close() for each name_open() of our name) or
				 * terminated
				 */
				ConnectDetach(ui_msg.hdr.scoid);
				break;
			case _PULSE_CODE_UNBLOCK:
				/*
				 * REPLY blocked client wants to unblock (was hit by a signal or timed out).  It's up to you if you
				 * reply now or later.
				 */
				break;
			default:
				if (ui_msg.hdr.code== ECP_TRIGGER) { // odebrano puls ECP_TRIGGER
					return true;
				}
				/*
				 * A pulse sent by one of your processes or a _PULSE_CODE_COIDDEATH or _PULSE_CODE_THREADDEATH
				 * from the kernel?
				 */
		}
	}

	if (rcvid > 0) {
		/* A QNX IO message received, reject */
		if (ui_msg.hdr.type >= _IO_BASE && ui_msg.hdr.type <= _IO_MAX) {
			MsgReply(rcvid, EOK, 0, 0);
		} else {
			/* A message (presumable ours) received, handle */
			printf("ECP trigger server receive strange message of type: %d\n", ui_msg.data);
			MsgReply(rcvid, EOK, 0, 0);
		}
	}

	return false;
}
// ---------------------------------------------------------------


void ecp_task::catch_signal_in_ecp_task(int sig)
{
	switch (sig) {
		case SIGTERM:
			kill_all_VSP(sensor_m);
			sr_ecp_msg->message("ECP terminated");
			_exit(EXIT_SUCCESS);
			break;
		case SIGSEGV:
			fprintf(stderr, "Segmentation fault in ECP process %s\n", config.section_name);
			signal(SIGSEGV, SIG_DFL);
			break;
	}
}

void ecp_task::terminate()
{
}

// ---------------------------------------------------------------
void ecp_task::initialize_communication()
{
	uint64_t e; // kod bledu systemowego

	char* sr_net_attach_point = config.return_attach_point_name(configurator::CONFIG_SERVER, "sr_attach_point", "[ui]");
	char* ecp_attach_point = config.return_attach_point_name(configurator::CONFIG_SERVER, "ecp_attach_point");
	char* trigger_attach_point = config.return_attach_point_name(configurator::CONFIG_SERVER, "trigger_attach_point");
	char* mp_pulse_attach_point =
			config.return_attach_point_name(configurator::CONFIG_SERVER, "mp_pulse_attach_point", "[mp]");

	if ((sr_ecp_msg = new sr_ecp(ECP, ecp_attach_point, sr_net_attach_point)) == NULL) { // Obiekt do komuniacji z SR
		perror("Unable to locate SR\n");
		throw ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	// Lokalizacja procesu MP - okreslenie identyfikatora (pid)
	if ( (MP_fd = name_open(mp_pulse_attach_point, NAME_FLAG_ATTACH_GLOBAL)) < 0) {
		e = errno;
		perror("ECP: Unable to locate MP_MASTER process\n");
		throw ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	// Rejstracja procesu ECP
	if ((ecp_attach = name_attach(NULL, ecp_attach_point, NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		e = errno;
		perror("Failed to attach Effector Control Process\n");
		sr_ecp_msg->message(SYSTEM_ERROR, e, "Failed to attach Effector Control Process");
		throw ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	if ((trigger_attach = name_attach(NULL, trigger_attach_point, NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		e = errno;
		perror("Failed to attach TRIGGER pulse chanel for ecp\n");
		sr_ecp_msg->message(SYSTEM_ERROR, e, "Failed  Failed to name attach (trigger pulse)");
		throw ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	delete [] ecp_attach_point;
	delete [] sr_net_attach_point;
	delete [] trigger_attach_point;
	delete [] mp_pulse_attach_point;

}
// -------------------------------------------------------------------


// methods for ECP template to redefine in concrete classes
void ecp_task::task_initialization(void)
{
}

void ecp_task::main_task_algorithm(void)
{
}

// Badanie typu polecenia z MP
MP_COMMAND ecp_task::mp_command_type(void) const
{
	return mp_command.command;
}

// Ustawienie typu odpowiedzi z ECP do MP
void ecp_task::set_ecp_reply(ECP_REPLY ecp_r)
{
	ecp_reply.reply = ecp_r;
}

// Informacja dla MP o zakonczeniu zadania uzytkownika
void ecp_task::ecp_termination_notice(void)
{

	if (mp_command_type()!= END_MOTION) {

		set_ecp_reply(TASK_TERMINATED);
		mp_buffer_receive_and_send();

	}
}

// Wysyla puls do Mp przed oczekiwaniem na spotkanie
void ecp_task::send_pulse_to_mp(int pulse_code, int pulse_value)
{
	if (MsgSendPulse(MP_fd, sched_get_priority_min(SCHED_FIFO), pulse_code, pulse_value) == -1) {
		perror("Blad w wysylaniu pulsu do mp\n");
	}
}

// Petla odbierania wiadomosci.
void ecp_task::ecp_wait_for_stop(void)
{
	// Wyslanie pulsu do MP
	send_pulse_to_mp(ECP_WAIT_FOR_STOP, 1);

	// Oczekiwanie na wiadomosc.
	int caller = receive_mp_message();

	if (mp_command_type() == STOP) {
		set_ecp_reply(ECP_ACKNOWLEDGE);
	} else {
		set_ecp_reply(ERROR_IN_ECP);
	}

	// Wyslanie odpowiedzi.
	if (MsgReply(caller, EOK, &ecp_reply, sizeof(ecp_reply)) ==-1) {// by Y&W
		uint64_t e= errno; // kod bledu systemowego
		perror("ECP: Reply to MP failed\n");
		sr_ecp_msg->message(SYSTEM_ERROR, e, "ECP: Reply to MP failed");
		throw ecp_generator::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	if (mp_command_type() != STOP) {
		fprintf(stderr, "ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d\n",
		__FILE__, __LINE__);
		throw ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}
}

// Oczekiwanie na polecenie START od MP
bool ecp_task::ecp_wait_for_start(void)
{
	bool ecp_stop = false;

	// Wyslanie pulsu do MP
	send_pulse_to_mp( ECP_WAIT_FOR_START, 1);

	int caller = receive_mp_message();

	switch (mp_command_type() ) {
		case START_TASK:
			// by Y - ECP_ACKNOWLEDGE zamienione na TASK_TERMINATED w celu uproszczenia oprogramowania zadan wielorobotowych
			set_ecp_reply(TASK_TERMINATED);
			break;
		case STOP:
			set_ecp_reply(TASK_TERMINATED);
			ecp_stop = true;
			break;
		default:
			set_ecp_reply(INCORRECT_MP_COMMAND);
			break;
	}

	// if (Reply (caller, &ecp_reply, sizeof(ECP_REPLY_PACKAGE)) == -1 ) {
	if (MsgReply(caller, EOK, &ecp_reply, sizeof(ECP_REPLY_PACKAGE)) ==-1) {// by Y&W
		uint64_t e= errno; // kod bledu systemowego
		perror("ECP: Reply to MP failed\n");
		sr_ecp_msg->message(SYSTEM_ERROR, e, "ECP: Reply to MP failed");
		throw ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}
	if (ecp_stop)
		throw ecp_generator::ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);

	if (ecp_reply.reply == INCORRECT_MP_COMMAND) {
		fprintf(stderr, "ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d\n",
		__FILE__, __LINE__);
		throw ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}

	sr_ecp_msg->message("ECP user program is running");
	return false;
}

// Oczekiwanie na kolejne zlecenie od MP
void ecp_task::get_next_state(void)
{
	bool ecp_stop = false;

	// Wyslanie pulsu do MP
	send_pulse_to_mp(ECP_WAIT_FOR_NEXT_STATE, 1);

	int caller = receive_mp_message();

	switch (mp_command_type() ) {
		case NEXT_STATE:
			set_ecp_reply(ECP_ACKNOWLEDGE);
			break;
		case STOP:
			set_ecp_reply(ECP_ACKNOWLEDGE);
			ecp_stop = true;
			break;
		default:
			set_ecp_reply(INCORRECT_MP_COMMAND);
			break;
	}

	if (MsgReply(caller, EOK, &ecp_reply, sizeof(ECP_REPLY_PACKAGE)) ==-1) {// by Y&W
		uint64_t e = errno; // kod bledu systemowego
		perror("ECP: Reply to MP failed\n");
		sr_ecp_msg->message(SYSTEM_ERROR, e, "ECP: Reply to MP failed");
		throw ECP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	if (ecp_stop)
		throw ecp_generator::ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);

	if (ecp_reply.reply == INCORRECT_MP_COMMAND) {
		fprintf(stderr, "ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d\n",
				__FILE__, __LINE__);
		throw ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}

	return;
}

// Oczekiwanie na polecenie od MP
bool ecp_task::mp_buffer_receive_and_send(void)
{
	bool returned_value = true;
	bool ecp_stop = false;

	// Wyslanie pulsu do MP
	send_pulse_to_mp(ECP_WAIT_FOR_COMMAND, 1);

	int caller = receive_mp_message();

	switch (mp_command_type()) {
		case NEXT_POSE:
			if ((ecp_reply.reply != TASK_TERMINATED)&&(ecp_reply.reply != ERROR_IN_ECP))
				set_ecp_reply(ECP_ACKNOWLEDGE);
			break;
		case STOP:
			set_ecp_reply(ECP_ACKNOWLEDGE);
			ecp_stop = true;
			break;
		case END_MOTION:
			// dla ulatwienia programowania apliakcji wielorobotowych
			if (ecp_reply.reply != ERROR_IN_ECP)
				set_ecp_reply(TASK_TERMINATED);
			returned_value = false;
			break;
		default:
			set_ecp_reply(INCORRECT_MP_COMMAND);
			break;
	}

	if (MsgReply(caller, EOK, &ecp_reply, sizeof(ecp_reply)) ==-1) {// by Y&W
		uint64_t e= errno; // kod bledu systemowego
		perror("ECP: Reply to MP failed\n");
		sr_ecp_msg->message(SYSTEM_ERROR, e, "ECP: Reply to MP failed");
		throw ecp_robot::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	if (ecp_stop)
		throw ecp_generator::ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);

	if (ecp_reply.reply == INCORRECT_MP_COMMAND) {
		fprintf(stderr, "ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND) @ %s:%d\n",
		__FILE__, __LINE__);
		throw ecp_generator::ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}

	return returned_value;
}

// Receive of mp message
int ecp_task::receive_mp_message(void)
{
	while (1) {

		int caller = MsgReceive(ecp_attach->chid, &mp_command, sizeof(mp_command), NULL);

		if (caller == -1) {/* Error condition, exit */
			uint64_t e= errno; // kod bledu systemowego
			perror("ECP: Receive from MP failed\n");
			sr_ecp_msg->message(SYSTEM_ERROR, e, "ECP: Receive from MP failed");
			throw ecp_robot::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
		}

		if (caller == 0) {/* Pulse received */
			switch (mp_command.hdr.code) {
				case _PULSE_CODE_DISCONNECT:
					/*
					 * A client disconnected all its connections (called
					 * name_close() for each name_open() of our name) or
					 * terminated
					 */
					ConnectDetach(mp_command.hdr.scoid);
					break;
				case _PULSE_CODE_UNBLOCK:
					/*
					 * REPLY blocked client wants to unblock (was hit by
					 * a signal or timed out).  It's up to you if you
					 * reply now or later.
					 */
					break;
				default:
					/*
					 * A pulse sent by one of your processes or a
					 * _PULSE_CODE_COIDDEATH or _PULSE_CODE_THREADDEATH
					 * from the kernel?
					 */
					break;
			}
			continue;
		}

		/* A QNX IO message received, reject */
		if (mp_command.hdr.type >= _IO_BASE && mp_command.hdr.type <= _IO_MAX) {
			MsgReply(caller, EOK, 0, 0);
			continue;
		}

		return caller;
	}
}

ecp_sub_task::ecp_sub_task(ecp_task &_ecp_t) :
	ecp_t(_ecp_t)
{
}

bool ecp_task::str_cmp::operator()(char const *a, char const *b) const
{
	return strcmp(a, b)<0;
}

mp::common::Trajectory * ecp_task::createTrajectory(xmlNode *actNode, xmlChar *stateID)
{
	mp::common::Trajectory* actTrajectory;
	xmlNode *cchild_node, *ccchild_node;
	xmlChar *coordinateType, *numOfPoses, *robot;
	xmlChar *xmlDataLine;
	//xmlChar *stateID, *stateType;

	coordinateType = xmlGetProp(actNode, (const xmlChar *)"coordinateType");
	numOfPoses = xmlGetProp(actNode, (const xmlChar *)"numOfPoses");
	actTrajectory = new mp::common::Trajectory((char *)numOfPoses, (char *)stateID, (char *)coordinateType);
	for(cchild_node = actNode->children; cchild_node!=NULL; cchild_node = cchild_node->next)
	{
		if ( cchild_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(cchild_node->name, (const xmlChar *)"Pose") )
		{
			actTrajectory->createNewPose();
			for(ccchild_node = cchild_node->children; ccchild_node!=NULL; ccchild_node = ccchild_node->next)
			{
				if ( ccchild_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(ccchild_node->name, (const xmlChar *)"StartVelocity") )
				{
					xmlDataLine = xmlNodeGetContent(ccchild_node);
					actTrajectory->setStartVelocities((char *)xmlDataLine);
					xmlFree(xmlDataLine);
				}
				if ( ccchild_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(ccchild_node->name, (const xmlChar *)"EndVelocity") )
				{
					xmlDataLine = xmlNodeGetContent(ccchild_node);
					actTrajectory->setEndVelocities((char *)xmlDataLine);
					xmlFree(xmlDataLine);
				}
				if ( ccchild_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(ccchild_node->name, (const xmlChar *)"Velocity") )
				{
					xmlDataLine = xmlNodeGetContent(ccchild_node);
					actTrajectory->setVelocities((char *)xmlDataLine);
					xmlFree(xmlDataLine);
				}
				if ( ccchild_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(ccchild_node->name, (const xmlChar *)"Accelerations") )
				{
					xmlDataLine = xmlNodeGetContent(ccchild_node);
					actTrajectory->setAccelerations((char *)xmlDataLine);
					xmlFree(xmlDataLine);
				}
				if ( ccchild_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(ccchild_node->name, (const xmlChar *)"Coordinates") )
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

	return actTrajectory;
}

std::map<char*, mp::common::Trajectory, ecp_task::str_cmp>* ecp_task::loadTrajectories(char * fileName, ROBOT_ENUM propRobot)
{
	int size;
	char * filePath;
	const char * robotName = mp::common::Trajectory::returnRobotName(propRobot);
	mp::common::Trajectory* actTrajectory;
	xmlNode *cur_node, *child_node, *subTaskNode;
	xmlChar *coordinateType, *numOfPoses, *robot;
	xmlChar *xmlDataLine;
	xmlChar *stateID, *stateType;
	std::map<char*, mp::common::Trajectory, str_cmp>* trajectoriesMap;

	xmlDocPtr doc;
	size = 1 + strlen(mrrocpp_network_path) + strlen(fileName);
	filePath = new char[size];

	// Stworzenie sciezki do pliku.
	sprintf(filePath, "%s%s", mrrocpp_network_path, fileName);
	doc = xmlParseFile(filePath);
	xmlXIncludeProcess(doc);
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

	trajectoriesMap = new std::map<char*, mp::common::Trajectory, ecp_task::str_cmp>();

   for(cur_node = root->children; cur_node != NULL; cur_node = cur_node->next)
   {
      if ( cur_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(cur_node->name, (const xmlChar *) "SubTask" ) )
		{
   		for(subTaskNode = cur_node->children; subTaskNode != NULL; subTaskNode = subTaskNode->next)
			{
				if ( subTaskNode->type == XML_ELEMENT_NODE  && !xmlStrcmp(subTaskNode->name, (const xmlChar *) "State" ) )
				{
					stateID = xmlGetProp(subTaskNode, (const xmlChar *) "id");
					stateType = xmlGetProp(subTaskNode, (const xmlChar *) "type");
					if(stateID && !strcmp((const char *)stateType, (const char *)"runGenerator"))
					{
						for(child_node = subTaskNode->children; child_node != NULL; child_node = child_node->next)
						{
							if ( child_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(child_node->name, (const xmlChar *)"ROBOT") )
							{
								robot = xmlNodeGetContent(child_node);
							}
							if ( child_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(child_node->name, (const xmlChar *)"Trajectory") &&
									!xmlStrcmp(robot, (const xmlChar *)robotName))
							{
								actTrajectory = createTrajectory(child_node, stateID);//new Trajectory((char *)numOfPoses, (char *)stateID, (char *)coordinateType);
								trajectoriesMap->insert(std::map<char *, mp::common::Trajectory>::value_type(actTrajectory->getTrjID(), *actTrajectory));
							}
						}
						xmlFree(robot);
					}
				}
			}
		}
      if ( cur_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(cur_node->name, (const xmlChar *) "State" ) )
      {
         stateID = xmlGetProp(cur_node, (const xmlChar *) "id");
			stateType = xmlGetProp(cur_node, (const xmlChar *) "type");
         if(stateID && !strcmp((const char *)stateType, (const char *)"runGenerator"))
			{
	         for(child_node = cur_node->children; child_node != NULL; child_node = child_node->next)
	         {
	            if ( child_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(child_node->name, (const xmlChar *)"ROBOT") )
					{
						robot = xmlNodeGetContent(child_node);
					}
	            if ( child_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(child_node->name, (const xmlChar *)"Trajectory") &&
							!xmlStrcmp(robot, (const xmlChar *)robotName))
	            {
						actTrajectory = createTrajectory(child_node, stateID);//new Trajectory((char *)numOfPoses, (char *)stateID, (char *)coordinateType);
						trajectoriesMap->insert(std::map<char *, mp::common::Trajectory>::value_type(actTrajectory->getTrjID(), *actTrajectory));
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

	return trajectoriesMap;
}
