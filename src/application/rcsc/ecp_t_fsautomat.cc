// -------------------------------------------------------------------------
//                            task/ecp_t_fsautomat.cc
//
// Funkcje do obslugi chwytaka, zacytowane z task/ecp_t_rcsc.cc
//
// Ostatnia modyfikacja: 2008
// -------------------------------------------------------------------------

#include <cstdio>
#include <cstring>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

#include "base/ecp/ecp_exceptions.h"
#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/ecp_mp/Trajectory.h"

#include "ecp_mp_t_fsautomat.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"

#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/tff_nose_run/ecp_mp_g_tff_nose_run.h"
#include "generator/ecp/tff_rubik_face_rotate/ecp_mp_g_tff_rubik_face_rotate.h"

#include "generator/ecp/bias_edp_force/ecp_g_bias_edp_force.h"
#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
#include "ecp_t_fsautomat.h"
#include "generator/ecp/bias_edp_force/ecp_g_bias_edp_force.h"

#include "generator/ecp/transparent/ecp_mp_g_transparent.h"
#include "generator/ecp/newsmooth/ecp_mp_g_newsmooth.h"
#include "generator/ecp/teach_in/ecp_mp_g_teach_in.h"
#include "generator/ecp/weight_measure/ecp_mp_g_weight_measure.h"
#include "../../base/ecp_mp/ecp_mp_task.h"

#include "base/lib/datastr.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

#if 0
void ecp_gripper_opening (task& _ecp_task, double gripper_increment, int motion_time)
{

	lib::trajectory_description tdes;

	tdes.arm_type = lib::WRIST_XYZ_EULER_ZYZ;
	tdes.interpolation_node_no = 1;
	tdes.internode_step_no = motion_time;
	tdes.value_in_step_no = tdes.internode_step_no - 2;
	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	tdes.coordinate_delta[0] = 0.0;// przyrost wspolrzednej X
	tdes.coordinate_delta[1] = 0.0;// przyrost wspolrzednej Y
	tdes.coordinate_delta[2] = 0.0;// przyrost wspolrzednej Z
	tdes.coordinate_delta[3] = 0.0;// przyrost wspolrzednej FI
	tdes.coordinate_delta[4] = 0.0;// przyrost wspolrzednej TETA
	tdes.coordinate_delta[5] = 0.0;// przyrost wspolrzednej PSI
	//	tdes.coordinate_delta[6] = 0.0;   // przyrost wspolrzednej PSI
	tdes.coordinate_delta[6] = gripper_increment;// przyrost wspolrzednej PSI

	// Generator trajektorii prostoliniowej
	ecp::common::generator::linear lg(_ecp_task, tdes, 1);

	_ecp_task.Move (lg);
}
#endif
// KONSTRUKTORY
fsautomat::fsautomat(lib::configurator &_config) :
		common::task::task(_config), gt(NULL), nrg(NULL), gag(NULL), rfrg(NULL), tig(NULL), befg(NULL), wmg(NULL)
{
	// the robot is choose dependendant on the section of configuration file sent as argv[4]
	if (config.robot_name == lib::irp6ot_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6ot_m::robot(*this);
	} else if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_m::robot(*this);
	} else {
		// TODO: throw, robot unsupported
		return;
	}
	std::cout << "INICJACJA ECP" << std::endl;
	const std::string whichECP = lib::toString(ecp_m_robot->robot_name);
	std::string filePath(mrrocpp_network_path);
	std::string fileName = config.value <std::string>("xml_file", "[xml_settings]");
	filePath += fileName;

	// open xml document
	xmlDocPtr doc = xmlParseFile(filePath.c_str());
	;
	if (doc == NULL) {
		fprintf(stderr, "ERROR in ecp initialization: could not parse file: %s\n", fileName.c_str());
		return;
	}

	// XML root
	xmlNode *root = xmlDocGetRootElement(doc);
	if (!root || !root->name) {
		fprintf(stderr, "ecp initialization ERROR: Bad root node name!");
		xmlFreeDoc(doc);
		return;
	}

	// for each root children "state"
	for (xmlNode *cur_node = root->children; cur_node != NULL; cur_node = cur_node->next) {
		if (cur_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cur_node->name, (const xmlChar *) "State")) {
			xmlChar * stateType = xmlGetProp(cur_node, (const xmlChar *) "type");
			if (!xmlStrcmp(stateType, (const xmlChar *) "systemInitialization")) {
				while (xmlStrcmp(cur_node->children->name, (const xmlChar *) "taskInit"))
					cur_node->children = cur_node->children->next;
				// For each child of state: i.e. Robot
				for (xmlNode *child_node = cur_node->children->children; child_node != NULL;
						child_node = child_node->next) {
					if (child_node->type == XML_ELEMENT_NODE && !xmlStrcmp(child_node->name, (const xmlChar *) "ecp")) {
						xmlChar * robot = xmlGetProp(child_node, (const xmlChar *) "name");
						if (robot && !xmlStrcmp(robot, (const xmlChar *) whichECP.c_str())) {
							for (; child_node->children; child_node->children = child_node->children->next) {
								if (child_node->children->type == XML_ELEMENT_NODE) {
									if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "ecp_gen_t")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											gt = new common::generator::transparent(*this);
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "ecp_tff_nose_run_gen")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											nrg = new common::generator::tff_nose_run(*this, atoi((char *) argument));
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "ecp_tff_gripper_approach_gen")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											gag =
													new common::generator::tff_gripper_approach(*this, atoi((char *) argument));
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "ecp_tff_rubik_face_rotate_gen")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											rfrg =
													new common::generator::tff_rubik_face_rotate(*this, atoi((char *) argument));
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "ecp_teach_in_gen")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) "")) {

										}
										tig = new common::generator::teach_in(*this);
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "bias_edp_force_gen")) {

										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) "")) {

										}
										befg = new common::generator::bias_edp_force(*this);
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "ecp_smooth_gen")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) "")) {
											axes_num = atoi((char *) argument);
											std::cout << "UWAGA!!!! liczba przegubow: !!!! " << axes_num << std::endl;
											sg = new common::generator::newsmooth(*this, lib::ECP_JOINT, axes_num); //changed askubis
										}
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "weight_measure_gen")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											wmg = new common::generator::weight_measure(*this, atoi((char *) argument));
										xmlFree(argument);
									}
								}
							}
						}
						xmlFree(robot);
					}
				}
			}
			xmlFree(stateType);
		}
	}
	xmlFreeDoc(doc);
	xmlCleanupParser();
}
//TODO: askubis dodac do XML definicje absolute/relative
void fsautomat::main_task_algorithm(void)
{
	std::cout << "MAIN_TASK ECP" << std::endl;
	std::string fileName = config.value <std::string>("xml_file", "[xml_settings]");
	int trjConf = config.value <int>("trajectory_from_xml", "[xml_settings]");
	int ecpLevel = config.value <int>("trajectory_on_ecp_level", "[xml_settings]");
	if (trjConf && ecpLevel) {
		std::cout << "!!!" << std::endl;
		std::cout << "trajectorymap" << std::endl;
		std::cout << "!!!" << std::endl;
		trjMap = loadTrajectories(fileName.c_str(), ecp_m_robot->robot_name, axes_num);
		printf("Lista %s zawiera: %zd elementow\n", lib::toString(ecp_m_robot->robot_name).c_str(), trjMap.size());
	}
	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		std::cout << "przed_next_state" << std::endl;
		get_next_state();
		std::cout << "po_next_state" << std::endl;
		sr_ecp_msg->message("Order received");

		subtasks_and_generators_dispatcher();
		std::cout << "NEXT STATE STRING OGOLNY        " << mp_2_ecp_next_state_string << std::endl;
		if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TEACH_IN) {
			std::string path(mrrocpp_network_path);
			path += (char*) mp_command.ecp_next_state.sg_buf.data;
			tig->flush_pose_list();
			//tig->load_file_with_path (path.c_str());
			//tig->initiate_pose_list();
			tig->teach(lib::ECP_MOTOR, "asdasdkjasdj");
			if (operator_reaction("Save?"))
				tig->save_file(lib::ECP_MOTOR);
			//tig->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_NEWSMOOTH) {
			if (trjConf) {
				if (ecpLevel) {
					std::cout << "armtype in fsautomat: " << (char*) mp_command.ecp_next_state.sg_buf.data << std::endl;
					std::cout << "NAZWASTANUUUUUUUUUUUU: " << (char*) mp_command.ecp_next_state.sg_buf.data
							<< std::endl;
					load_trajectory_from_xml(trjMap[(std::string) (char*) mp_command.ecp_next_state.sg_buf.data]);
				} else {
					std::string path(mrrocpp_network_path);
					path += fileName;
					load_trajectory_from_xml(path.c_str(), (char*) mp_command.ecp_next_state.sg_buf.data);
				}
			} //if
			else //moj przypadekl -> z pliku
			{
				std::string path(mrrocpp_network_path);
				if (((char*) mp_command.ecp_next_state.sg_buf.data)[0] == '/') {
					path = (char*) mp_command.ecp_next_state.sg_buf.data;
				} else {
					path += (char*) mp_command.ecp_next_state.sg_buf.data;
				}
				//std::cout<<"WCZYTYWANIE Z PLIKU, SCIEZKA:   "<<path<< " VARIANT "<<mp_command.ecp_next_state.mp_2_ecp_next_state_variant<<std::endl;
				//std::cout<<"STATE:  "<<mp_command.ecp_next_state.mp_2_ecp_next_state<<" STRING:  "<<(char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string<<std::endl;
				sg->load_trajectory_from_file(path.c_str());
			} //else
			  //std::cout<<"interpolating"<<std::endl;
			sg->calculate_interpolate();
			//std::cout<<"interpolated, moving"<<std::endl;
			sg->Move(); //changed askubis
		} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_WEIGHT_MEASURE) {
			wmg->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TRANSPARENT) {
			gt->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN) {
			nrg->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE) {
			befg->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_RUBIK_FACE_ROTATE) {
			double gen_args[1];
			lib::setValuesInArray(gen_args, (char*) mp_command.ecp_next_state.sg_buf.data);
			rfrg->configure(gen_args[0]);
			rfrg->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH) {
			//double gen_args[2];//TODO@ askubis zmienic spowrotem
			//std::cout<<"gag"<<(char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string<<std::endl;
			//std::cout<<"gag"<<(char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string<<std::endl;

			//lib::setValuesInArray(gen_args, (char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string);
			//std::cout<<"gag"<<std::endl;
			gag->configure(0.02, 300, 3);
			//std::cout<<"gag_configured"<<std::endl;
			gag->Move();
			//std::cout<<"gag_configured moved"<<std::endl;
			//lib::STOP czyli 2 powinien byc a jest 4...
		}

		termination_notice();
	} //end for

}

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new fsautomat(_config);
}

void fsautomat::load_trajectory_from_xml(std::pair <
		std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose *>, lib::MOTION_TYPE> pair_trj_motion)
{
	std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose *> trj_vect = pair_trj_motion.first;
	if (pair_trj_motion.second == lib::ABSOLUTE)
		for (int i = 0; i < trj_vect.size(); i++) {
			std::cout << "WEKTOR ZAWIERA " << trj_vect.size() << " ELEMENTOW" << std::endl;
			ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose trj = (*trj_vect[i]);
			sg->load_absolute_pose(trj);
		}
	else if (pair_trj_motion.second == lib::RELATIVE)
		for (int i = 0; i < trj_vect.size(); i++) {
			std::cout << "WEKTOR ZAWIERA " << trj_vect.size() << " ELEMENTOW" << std::endl;
			ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose trj = (*trj_vect[i]);
			sg->load_relative_pose(trj);
		}
}

void fsautomat::load_trajectory_from_xml(const char* fileName, const char* nodeName)
{
	// Funkcja zwraca true jesli wczytanie trajektorii powiodlo sie,

	bool first_time = true; // Znacznik

	xmlDocPtr doc = xmlParseFile(fileName);
	xmlXIncludeProcess(doc);
	if (doc == NULL) {
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(NON_EXISTENT_FILE));
	}

	xmlNodePtr root = xmlDocGetRootElement(doc);
	if (!root || !root->name) {
		xmlFreeDoc(doc);
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
	}

	//sg->reset(); // Usuniecie listy pozycji, o ile istnieje

	for (xmlNodePtr cur_node = root->children; cur_node != NULL; cur_node = cur_node->next) {
		if (cur_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cur_node->name, (const xmlChar *) "SubTask")) {
			for (xmlNodePtr subTaskNode = cur_node->children; subTaskNode != NULL; subTaskNode = subTaskNode->next) {
				if (subTaskNode->type == XML_ELEMENT_NODE && !xmlStrcmp(subTaskNode->name, (const xmlChar *) "State")) {
					xmlChar * stateID = xmlGetProp(subTaskNode, (const xmlChar *) "id");
					if (stateID && !strcmp((const char *) stateID, nodeName)) {
						for (xmlNodePtr child_node = subTaskNode->children; child_node != NULL;
								child_node = child_node->next) {
							if (child_node->type == XML_ELEMENT_NODE
									&& !xmlStrcmp(child_node->name, (const xmlChar *) "Trajectory")) {
								set_pose_from_xml(child_node, first_time);
							}
						}
					}
					xmlFree(stateID);
				}
			}
		}
		if (cur_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cur_node->name, (const xmlChar *) "State")) {
			xmlChar * stateID = xmlGetProp(cur_node, (const xmlChar *) "id");
			if (stateID && !strcmp((const char *) stateID, nodeName)) {
				for (xmlNodePtr child_node = cur_node->children; child_node != NULL; child_node = child_node->next) {
					if (child_node->type == XML_ELEMENT_NODE
							&& !xmlStrcmp(child_node->name, (const xmlChar *) "Trajectory")) {
						set_pose_from_xml(child_node, first_time);
					}
				}
			}
			xmlFree(stateID);
		}
	}
	xmlFreeDoc(doc);
	xmlCleanupParser();
}

void fsautomat::set_pose_from_xml(xmlNode *stateNode, bool &first_time)
{
	lib::ECP_POSE_SPECIFICATION ps; // Rodzaj wspolrzednych

	xmlNode *cchild_node, *ccchild_node;
	xmlChar *coordinateType, *m_type;
	xmlChar *xmlDataLine;

	int num = 0;

	coordinateType = xmlGetProp(stateNode, (const xmlChar *) "coordinateType");
	ps = lib::returnProperPS((char *) coordinateType);
	m_type = xmlGetProp(stateNode, (const xmlChar *) "motionType");
	char * motionType = ((char *) m_type);

	double tmp[10];

	for (cchild_node = stateNode->children; cchild_node != NULL; cchild_node = cchild_node->next) {
		if (cchild_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cchild_node->name, (const xmlChar *) "Pose")) {
			ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose * actTrajectory =
					new ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose();
			actTrajectory->arm_type = ps;
			for (ccchild_node = cchild_node->children; ccchild_node != NULL; ccchild_node = ccchild_node->next) {
				if (ccchild_node->type == XML_ELEMENT_NODE
						&& !xmlStrcmp(ccchild_node->name, (const xmlChar *) "Velocity")) {
					xmlDataLine = xmlNodeGetContent(ccchild_node);
					num = lib::setValuesInArray(tmp, (char *) xmlDataLine);
					for (int i = 0; i < num; i++) {
						actTrajectory->v.push_back(tmp[i]);
					}
					xmlFree(xmlDataLine);
				}
				if (ccchild_node->type == XML_ELEMENT_NODE
						&& !xmlStrcmp(ccchild_node->name, (const xmlChar *) "Accelerations")) {
					xmlDataLine = xmlNodeGetContent(ccchild_node);
					num = lib::setValuesInArray(tmp, (char *) xmlDataLine);
					for (int i = 0; i < num; i++) {
						actTrajectory->a.push_back(tmp[i]);
					}
					xmlFree(xmlDataLine);
				}
				if (ccchild_node->type == XML_ELEMENT_NODE
						&& !xmlStrcmp(ccchild_node->name, (const xmlChar *) "Coordinates")) {
					xmlDataLine = xmlNodeGetContent(ccchild_node);
					num = lib::setValuesInArray(tmp, (char *) xmlDataLine);
					for (int i = 0; i < num; i++) {
						actTrajectory->coordinates.push_back(tmp[i]);
					}
					xmlFree(xmlDataLine);
				}
				/*for (int i = 0; i<actTrajectory->coordinates.size(); i++)
				 {
				 std::cout<<"COORDS: "<<actTrajectory->coordinates[i]<<std::endl;
				 std::cout<<"SPEED:  "<<actTrajectory->v[i]<<std::endl;
				 std::cout<<"ACCEL:  "<<actTrajectory->a[i]<<std::endl;
				 }*/
			}
			if (!strcmp(motionType, "Absolute"))
				sg->load_absolute_pose((*actTrajectory));
			else if (!strcmp(motionType, "Realative"))
				sg->load_relative_pose((*actTrajectory));
			else {
				//some error
			}

		}
	}
	xmlFree(coordinateType);
	xmlFree(m_type);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
