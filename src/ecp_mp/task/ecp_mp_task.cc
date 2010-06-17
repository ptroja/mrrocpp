// -------------------------------------------------------------------------
//                            ecp_mp_task.cc
//            Effector Control Process (lib::ECP) i MP - methods
//
// -------------------------------------------------------------------------

#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#if defined(__QNXNTO__)
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#endif /* __QNXNTO__ */

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/datastr.h"
#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_task.h"
#include "ecp_mp/sensor/ecp_mp_sensor.h"
#include "ecp/common/ECP_main_error.h"

#if defined(USE_MESSIP_SRR)
#include "messip_dataport.h"
#endif

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xinclude.h>

#include <boost/foreach.hpp>

namespace mrrocpp {
namespace ecp_mp {
namespace task {

lib::sr_ecp* task::sr_ecp_msg = NULL;
lib::sr_ecp* task::sh_msg = NULL;


task::task(lib::configurator &_config)
	: config(_config),
	mrrocpp_network_path(config.return_mrrocpp_network_path())
{
	// std::string sr_net_attach_point = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", UI_SECTION);

//	// Obiekt do komuniacji z SR
//	sr_ecp_msg = new lib::sr_ecp(process_type, process_name, sr_net_attach_point);

	const std::string ui_net_attach_point = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "ui_attach_point", UI_SECTION);

    // kilka sekund  (~1) na otworzenie urzadzenia
    short tmp = 0;
#if !defined(USE_MESSIP_SRR)
    while ((UI_fd = name_open(ui_net_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) < 0) {
#else
	while ((UI_fd = messip::port_connect(ui_net_attach_point)) == NULL) {
#endif
        if ((tmp++)<CONNECT_RETRY)
            usleep(1000*CONNECT_DELAY);
        else
        {
            int e = errno;
            perror("Connect to UI failed");
            // WARNING: sr_ecp_msg is not yet initialized!;
            // it will be created in ecp_task/mp_task constructors

            // sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "Connect to UI failed");
            throw ecp_mp::task::ECP_MP_main_error(lib::SYSTEM_ERROR, e	);
        }
    }
}

task::~task()
{
	// Zabicie wszystkich procesow VSP
	BOOST_FOREACH(sensor_item_t & sensor_item, sensor_m) {
		delete sensor_item.second;
	}
}


// --------------------------------------------------------------------------
// Odpowiedz operatora typu (Yes/No) na zadane pytanie (question)
bool task::operator_reaction (const char* question )
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::YES_NO;     // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI podczas uczenia

#if !defined(USE_MESSIP_SRR)
	ecp_to_ui_msg.hdr.type=0;
	if (MsgSend(UI_fd, &ecp_to_ui_msg, sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0) {// by Y&W
#else
	if(messip::port_send(UI_fd, 0, 0, &ecp_to_ui_msg, ui_to_ecp_rep) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP operator_reaction(): Send() to UI failed");
		sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(lib::SYSTEM_ERROR, 0);
	}

	return (ui_to_ecp_rep.reply == lib::ANSWER_YES);
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// by Y - Wybor przez operatora jednej z opcji
uint8_t task::choose_option (const char* question, uint8_t nr_of_options_input )
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::CHOOSE_OPTION; // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI
	ecp_to_ui_msg.nr_of_options = nr_of_options_input;

#if !defined(USE_MESSIP_SRR)
	ecp_to_ui_msg.hdr.type=0;
	if (MsgSend(UI_fd, &ecp_to_ui_msg, sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0) {// by Y&W
#else
	if(messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP: Send() to UI failed");
		sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(lib::SYSTEM_ERROR, 0);
	}

	return ui_to_ecp_rep.reply; // by Y
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Zadanie od operatora podania liczby calkowitej (int)
int task::input_integer (const char* question )
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::INTEGER_NUMBER; // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI

#if !defined(USE_MESSIP_SRR)
	ecp_to_ui_msg.hdr.type=0;
	if (MsgSend(UI_fd, &ecp_to_ui_msg,  sizeof(lib::ECP_message),  &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0) {// by Y&W
#else
	if(messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP: Send() to UI failed");
		sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(lib::SYSTEM_ERROR, 0);
	}

	return ui_to_ecp_rep.integer_number;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Zadanie od operatora podania liczby rzeczywistej (double)
double task::input_double (const char* question )
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::DOUBLE_NUMBER; // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI

#if !defined(USE_MESSIP_SRR)
	ecp_to_ui_msg.hdr.type=0;
	if (MsgSend(UI_fd, &ecp_to_ui_msg, sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0) {// by Y&W
#else
	if(messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP: Send() to UI failed");
		sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(lib::SYSTEM_ERROR, 0);
	}
	return ui_to_ecp_rep.double_number; // by Y
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Informacja wymagajaca potwierdzenia odbioru przez operatora
bool task::show_message (const char* message)
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::MESSAGE; // Polecenie wyswietlenia komunikatu
	strcpy(ecp_to_ui_msg.string, message);

#if !defined(USE_MESSIP_SRR)
	ecp_to_ui_msg.hdr.type=0;
	if (MsgSend(UI_fd, &ecp_to_ui_msg,  sizeof(lib::ECP_message),  &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0) {// by Y&W
#else
	if(messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {
#endif
		uint64_t e = errno;
		perror("ECP: Send() to UI failed");
		sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw ECP_MP_main_error(lib::SYSTEM_ERROR, 0);
	}

	return (ui_to_ecp_rep.reply == lib::ANSWER_YES);
}
// --------------------------------------------------------------------------


// Funkcje do obslugi czujnikow
void task::all_sensors_initiate_reading (sensors_t & _sensor_m)
{
	BOOST_FOREACH(sensor_item_t & sensor_item, _sensor_m) {
		if (sensor_item.second->base_period > 0) {
			if (sensor_item.second->current_period == sensor_item.second->base_period) {
				sensor_item.second->initiate_reading();
			}
			sensor_item.second->current_period--;
		}
	}
}

void task::all_sensors_get_reading (sensors_t & _sensor_m)
{
	BOOST_FOREACH(sensor_item_t & sensor_item, _sensor_m) {
		// jesli wogole mamy robic pomiar
		if (sensor_item.second->base_period > 0) {
			if (sensor_item.second->current_period == 0) {
				sensor_item.second->get_reading();
				sensor_item.second->current_period = sensor_item.second->base_period;
			}
		}
	}
}

bool task::str_cmp::operator()(char const *a, char const *b) const
{
	return strcmp(a, b)<0;
}

ecp_mp::common::Trajectory * task::createTrajectory2(xmlNodePtr actNode, xmlChar *stateID)
{
	xmlChar * coordinateType = xmlGetProp(actNode, (const xmlChar *)"coordinateType");
	xmlChar * numOfPoses = xmlGetProp(actNode, (const xmlChar *)"numOfPoses");

	ecp_mp::common::Trajectory * actTrajectory = new ecp_mp::common::Trajectory((char *)numOfPoses, (char *)stateID, (char *)coordinateType);

	for(xmlNodePtr cchild_node = actNode->children; cchild_node!=NULL; cchild_node = cchild_node->next)
	{
		if ( cchild_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cchild_node->name, (const xmlChar *)"Pose") )
		{
			actTrajectory->createNewPose();
			for(xmlNodePtr ccchild_node = cchild_node->children; ccchild_node!=NULL; ccchild_node = ccchild_node->next)
			{
				if ( ccchild_node->type == XML_ELEMENT_NODE ) {
					if ( !xmlStrcmp(ccchild_node->name, (const xmlChar *)"Velocity") )
					{
						xmlChar *xmlDataLine = xmlNodeGetContent(ccchild_node);
						actTrajectory->setVelocities((char *)xmlDataLine);
						xmlFree(xmlDataLine);
					}
					else if ( !xmlStrcmp(ccchild_node->name, (const xmlChar *)"Accelerations") )
					{
						xmlChar *xmlDataLine = xmlNodeGetContent(ccchild_node);
						actTrajectory->setAccelerations((char *)xmlDataLine);
						xmlFree(xmlDataLine);
					}
					else if ( !xmlStrcmp(ccchild_node->name, (const xmlChar *)"Coordinates") )
					{
						xmlChar *xmlDataLine = xmlNodeGetContent(ccchild_node);
						actTrajectory->setCoordinates((char *)xmlDataLine);
						xmlFree(xmlDataLine);
					}
				}
			}
			actTrajectory->addPoseToTrajectory();
		}

	}
	xmlFree(coordinateType);
	xmlFree(numOfPoses);

	return actTrajectory;
}

task::trajectories_t * task::loadTrajectories(const char * fileName, lib::robot_name_t propRobot)
{
	// Stworzenie sciezki do pliku.
	std::string filePath(mrrocpp_network_path);
	filePath += fileName;

	xmlDocPtr doc = xmlParseFile(filePath.c_str());

	if(doc == NULL)
	{
		fprintf(stderr, "loadTrajectories NON_EXISTENT_FILE\n");
		// throw ecp::common::generator::generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	if(xmlXIncludeProcess(doc) < 0) {
		fprintf(stderr, "loadTrajectories XInclude failed\n");
		// throw ecp::common::generator::generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	xmlNodePtr root = xmlDocGetRootElement(doc);
	if(!root || !root->name)
	{
		xmlFreeDoc(doc);
		printf("loadTrajectories READ_FILE_ERROR\n");
		// throw ecp::common::generator::generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
	}

	trajectories_t* trajectoriesMap = new trajectories_t();

	const std::string robotName(lib::toString(propRobot));

	for(xmlNodePtr cur_node = root->children; cur_node != NULL; cur_node = cur_node->next)
	{
		if ( cur_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cur_node->name, (const xmlChar *) "SubTask" ) )
		{
			for(xmlNodePtr subTaskNode = cur_node->children; subTaskNode != NULL; subTaskNode = subTaskNode->next)
			{
				if ( subTaskNode->type == XML_ELEMENT_NODE  && !xmlStrcmp(subTaskNode->name, (const xmlChar *) "State" ) )
				{
					xmlChar * stateID = xmlGetProp(subTaskNode, (const xmlChar *) "id");
					xmlChar * stateType = xmlGetProp(subTaskNode, (const xmlChar *) "type");
					if(stateID && !strcmp((const char *)stateType, (const char *)"runGenerator"))
					{
						xmlChar *robot = NULL;
						for(xmlNodePtr child_node = subTaskNode->children; child_node != NULL; child_node = child_node->next)
						{
							if ( child_node->type == XML_ELEMENT_NODE && !xmlStrcmp(child_node->name, (const xmlChar *)"ROBOT") )
							{
								robot = xmlNodeGetContent(child_node);
							}
							if ( child_node->type == XML_ELEMENT_NODE && !xmlStrcmp(child_node->name, (const xmlChar *)"Trajectory") &&
									!xmlStrcmp(robot, (const xmlChar *)robotName.c_str()))
							{
								ecp_mp::common::Trajectory* actTrajectory = createTrajectory2(child_node, stateID);//new Trajectory((char *)numOfPoses, (char *)stateID, (char *)coordinateType);
								trajectoriesMap->insert(trajectories_t::value_type(actTrajectory->getTrjID().c_str(), *actTrajectory));
							}
						}
						xmlFree(robot);
					}
				}
			}
		}
		else if (cur_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cur_node->name, (const xmlChar *) "State")) {
			xmlChar * stateID = xmlGetProp(cur_node, (const xmlChar *) "id");
			xmlChar * stateType = xmlGetProp(cur_node, (const xmlChar *) "type");
			if (stateID && !strcmp((const char *) stateType, "runGenerator")) {
				xmlChar *robot = NULL;
				for (xmlNodePtr child_node = cur_node->children; child_node != NULL; child_node = child_node->next) {
					if (child_node->type == XML_ELEMENT_NODE && !xmlStrcmp(
							child_node->name, (const xmlChar *) "ROBOT")) {
						robot = xmlNodeGetContent(child_node);
					}
					if (child_node->type == XML_ELEMENT_NODE && !xmlStrcmp(
							child_node->name, (const xmlChar *) "Trajectory")
							&& !xmlStrcmp(robot, (const xmlChar *) robotName.c_str())) {
						ecp_mp::common::Trajectory* actTrajectory = createTrajectory2(child_node, stateID);
						trajectoriesMap->insert(trajectories_t::value_type(actTrajectory->getTrjID().c_str(), *actTrajectory));
					}
				}
				xmlFree(robot);
			}
		}
	}
	xmlFreeDoc(doc);
	xmlCleanupParser();
	//	for(trajectories_t::iterator ii = trjMap->begin(); ii != trjMap->end(); ++ii)
	//		(*ii).second.showTime();

	return trajectoriesMap;
}

} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp
