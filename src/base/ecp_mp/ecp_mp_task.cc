/*!
 * @file
 * @brief File contains ecp_mp base task definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp_mp
 */

#include <cstdio>
#include <iostream>
#include <stdint.h>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <csignal>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xinclude.h>

#include "base/lib/datastr.h"

#include "base/ecp_mp/ecp_ui_msg.h"
#include "ecp_mp_exceptions.h"
#include "ecp_mp_task.h"
#include "ecp_mp_sensor.h"

#include "base/lib/agent/Agent.h"
#include "base/lib/messip/messip_dataport.h"

namespace mrrocpp {
namespace ecp_mp {
namespace task {

boost::shared_ptr <lib::sr_ecp> task::sr_ecp_msg;

task::task(lib::configurator &_config) :
		Agent(_config.section_name), config(_config), mrrocpp_network_path(config.return_mrrocpp_network_path())
{
	const std::string ui_net_attach_point = config.get_ui_attach_point();

	// kilka sekund  (~1) na otworzenie urzadzenia
	unsigned int tmp = 0;

	while ((UI_fd = messip::port_connect(ui_net_attach_point)) == NULL) {
		if ((tmp++) < lib::CONNECT_RETRY) {
			boost::this_thread::sleep(lib::CONNECT_DELAY);
		} else {
			int e = errno;
			perror("Connect to UI failed");
			// WARNING: sr_ecp_msg is not yet initialized!;
			// it will be created in ecp_task/mp_task constructors

			// sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "Connect to UI failed");
			BOOST_THROW_EXCEPTION(exception::se() << lib::exception::mrrocpp_error0(e));
		}
	}
}

task::~task()
{
	// Zabicie wszystkich procesow VSP
	BOOST_FOREACH(sensor_item_t & sensor_item, sensor_m)
			{
				delete sensor_item.second;
			}
}

// --------------------------------------------------------------------------
// Odpowiedz operatora typu (Yes/No) na zadane pytanie (question)
bool task::operator_reaction(const char* question)
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::YES_NO; // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI podczas uczenia

	if (messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {

		uint64_t e = errno;
		perror("ecp operator_reaction(): Send() to UI failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		BOOST_THROW_EXCEPTION(exception::se());
	}

	return (ui_to_ecp_rep.reply == lib::ANSWER_YES);
}
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// by Y - Wybor przez operatora jednej z opcji
uint8_t task::choose_option(const char* question, uint8_t nr_of_options_input)
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::CHOOSE_OPTION; // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI
	ecp_to_ui_msg.nr_of_options = nr_of_options_input;

	if (messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {

		uint64_t e = errno;
		perror("ecp: Send() to UI failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		BOOST_THROW_EXCEPTION(exception::se());
	}

	return ui_to_ecp_rep.reply; // by Y
}
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Zadanie od operatora podania liczby calkowitej (int)
int task::input_integer(const char* question)
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::INTEGER_NUMBER; // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI

	if (messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {

		uint64_t e = errno;
		perror("ecp: Send() to UI failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		BOOST_THROW_EXCEPTION(exception::se());
	}

	return ui_to_ecp_rep.integer_number;
}
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Zadanie od operatora podania liczby rzeczywistej (double)
double task::input_double(const char* question)
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::DOUBLE_NUMBER; // Polecenie odpowiedzi na zadane
	strcpy(ecp_to_ui_msg.string, question); // Komunikat przesylany do UI

	if (messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {
		uint64_t e = errno;
		perror("ecp: Send() to UI failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		BOOST_THROW_EXCEPTION(exception::se());
	}
	return ui_to_ecp_rep.double_number; // by Y
}
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Informacja wymagajaca potwierdzenia odbioru przez operatora
bool task::show_message(const char* message)
{
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep; // Odpowiedz UI do ECP

	ecp_to_ui_msg.ecp_message = lib::MESSAGE; // Polecenie wyswietlenia komunikatu
	strcpy(ecp_to_ui_msg.string, message);

	if (messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {
		uint64_t e = errno;
		perror("ecp: Send() to UI failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		BOOST_THROW_EXCEPTION(exception::se());
	}

	return (ui_to_ecp_rep.reply == lib::ANSWER_YES);
}

ecp_mp::common::trajectory_pose::bang_bang_motion_trajectory task::createTrajectory2(xmlNodePtr actNode, xmlChar *stateID, int axes_num)
{
	xmlChar * coordinateType = xmlGetProp(actNode, (const xmlChar *) "coordinateType");
	xmlChar * m_type = xmlGetProp(actNode, (const xmlChar *) "motionType");
	std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose *> trj_vect;
	/*ecp_mp::common::trajectory_pose::trajectory_pose * actTrajectory =
	 new ecp_mp::common::trajectory_pose::trajectory_pose((char *) numOfPoses, (char *) stateID, (char *) coordinateType);*/

	//coordinateType wrzucic do
	double tmp[10]; //TODO: askubis check if it's enough
	int num = 0;

	for (xmlNodePtr cchild_node = actNode->children; cchild_node != NULL; cchild_node = cchild_node->next) {
		if (cchild_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cchild_node->name, (const xmlChar *) "Pose")) {
			ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose * actTrajectory =
					new ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose();
			actTrajectory->arm_type = lib::returnProperPS((char *) coordinateType);
			for (xmlNodePtr ccchild_node = cchild_node->children; ccchild_node != NULL;
					ccchild_node = ccchild_node->next) {
				if (ccchild_node->type == XML_ELEMENT_NODE) {
					if (!xmlStrcmp(ccchild_node->name, (const xmlChar *) "Velocity")) {
						xmlChar *xmlDataLine = xmlNodeGetContent(ccchild_node);
						num = lib::setValuesInArray(tmp, (char *) xmlDataLine);
						for (int i = 0; i < num; i++) {
							actTrajectory->v.push_back(tmp[i]);
						}
						xmlFree(xmlDataLine);
					} else if (!xmlStrcmp(ccchild_node->name, (const xmlChar *) "Accelerations")) {
						xmlChar *xmlDataLine = xmlNodeGetContent(ccchild_node);
						num = lib::setValuesInArray(tmp, (char *) xmlDataLine);
						for (int i = 0; i < num; i++) {
							actTrajectory->a.push_back(tmp[i]);
						}
						xmlFree(xmlDataLine);
					} else if (!xmlStrcmp(ccchild_node->name, (const xmlChar *) "Coordinates")) {
						xmlChar *xmlDataLine = xmlNodeGetContent(ccchild_node);
						num = lib::setValuesInArray(tmp, (char *) xmlDataLine);
						for (int i = 0; i < num; i++) {
							actTrajectory->coordinates.push_back(tmp[i]);
						}
						xmlFree(xmlDataLine);
					}
				}
			}
			trj_vect.push_back(actTrajectory);
		}

	}
	xmlFree(coordinateType);
	xmlFree(m_type);
	if (!strcmp((char *) m_type, "Absoulte"))
		return ecp_mp::common::trajectory_pose::bang_bang_motion_trajectory(trj_vect, lib::ABSOLUTE);
	else if (!strcmp((char *) m_type, "Relative"))
		return ecp_mp::common::trajectory_pose::bang_bang_motion_trajectory(trj_vect, lib::RELATIVE);
	else {
		return ecp_mp::common::trajectory_pose::bang_bang_motion_trajectory(trj_vect, lib::ABSOLUTE); //default
	}

}

task::bang_trajectories_map task::loadTrajectories(const char * fileName, lib::robot_name_t propRobot, int axes_num)
{ //boost pointermap
// Stworzenie sciezki do pliku.
	std::string filePath(mrrocpp_network_path);
	filePath += fileName;

	xmlDocPtr doc = xmlParseFile(filePath.c_str());

	if (doc == NULL) {
		fprintf(stderr, "loadTrajectories NON_EXISTENT_FILE\n");
		// throw ecp::common::generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	if (xmlXIncludeProcess(doc) < 0) {
		fprintf(stderr, "loadTrajectories XInclude failed\n");
		// throw ecp::common::generator::ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	xmlNodePtr root = xmlDocGetRootElement(doc);
	if (!root || !root->name) {
		xmlFreeDoc(doc);
		printf("loadTrajectories READ_FILE_ERROR\n");
		// throw ecp::common::generator::ECP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
	}

	bang_trajectories_map trajectoriesMap;

	const std::string robotName(lib::toString(propRobot));

	for (xmlNodePtr cur_node = root->children; cur_node != NULL; cur_node = cur_node->next) {
		if (cur_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cur_node->name, (const xmlChar *) "SubTask")) {
			for (xmlNodePtr subTaskNode = cur_node->children; subTaskNode != NULL; subTaskNode = subTaskNode->next) {
				if (subTaskNode->type == XML_ELEMENT_NODE && !xmlStrcmp(subTaskNode->name, (const xmlChar *) "State")) {
					xmlChar * stateID = xmlGetProp(subTaskNode, (const xmlChar *) "id");
					xmlChar * stateType = xmlGetProp(subTaskNode, (const xmlChar *) "type");
					if (stateID && !strcmp((const char *) stateType, (const char *) "runGenerator")) {
						xmlChar *robot = NULL;
						for (xmlNodePtr child_node = subTaskNode->children; child_node != NULL;
								child_node = child_node->next) {
							if (child_node->type == XML_ELEMENT_NODE
									&& !xmlStrcmp(child_node->name, (const xmlChar *) "ROBOT")) {
								robot = xmlNodeGetContent(child_node);
							}
							if (child_node->type == XML_ELEMENT_NODE
									&& !xmlStrcmp(child_node->name, (const xmlChar *) "Trajectory")
									&& !xmlStrcmp(robot, (const xmlChar *) robotName.c_str())) {
								ecp_mp::common::trajectory_pose::bang_bang_motion_trajectory trj_motion =
										createTrajectory2(child_node, stateID, axes_num);
								trajectoriesMap.insert(bang_trajectories_map::value_type((std::string) (char *) stateID, (trj_motion)));
							}
						}

						xmlFree(robot);
					}
				}
			}
		} else if (cur_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cur_node->name, (const xmlChar *) "State")) {
			xmlChar * stateID = xmlGetProp(cur_node, (const xmlChar *) "id");
			xmlChar * stateType = xmlGetProp(cur_node, (const xmlChar *) "type");
			if (stateID && !strcmp((const char *) stateType, "runGenerator")) {
				xmlChar *robot = NULL;
				for (xmlNodePtr child_node = cur_node->children; child_node != NULL; child_node = child_node->next) {
					if (child_node->type == XML_ELEMENT_NODE
							&& !xmlStrcmp(child_node->name, (const xmlChar *) "ROBOT")) {
						robot = xmlNodeGetContent(child_node);
					}
					if (child_node->type == XML_ELEMENT_NODE
							&& !xmlStrcmp(child_node->name, (const xmlChar *) "Trajectory")
							&& !xmlStrcmp(robot, (const xmlChar *) robotName.c_str())) {
						ecp_mp::common::trajectory_pose::bang_bang_motion_trajectory trj_motion = createTrajectory2(child_node, stateID, axes_num);
						trajectoriesMap.insert(bang_trajectories_map::value_type((std::string) (char *) stateID, (trj_motion)));
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
