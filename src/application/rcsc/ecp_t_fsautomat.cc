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

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/ecp_mp/Trajectory.h"

#include "base/lib/sr/srlib.h"
#include "ecp_mp_t_fsautomat.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "generator/ecp/ecp_g_newsmooth.h"

#include "generator/ecp/force/ecp_mp_g_bias_edp_force.h"
#include "generator/ecp/force/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/force/ecp_mp_g_tff_nose_run.h"
#include "generator/ecp/force/ecp_mp_g_tff_rubik_face_rotate.h"
#include "generator/ecp/force/ecp_mp_g_tff_rubik_grab.h"
#include "generator/ecp/force/ecp_g_bias_edp_force.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"
#include "ecp_t_fsautomat.h"
#include "subtask/ecp_st_bias_edp_force.h"
#include "subtask/ecp_st_tff_nose_run.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"
#include "generator/ecp/ecp_mp_g_teach_in.h"
#include "generator/ecp/force/ecp_mp_g_weight_measure.h"
#include "subtask/ecp_mp_st_gripper_opening.h"

#include "base/lib/datastr.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

#if 0
void ecp_gripper_opening (task& _ecp_task, double gripper_increment, int motion_time)
{

	lib::trajectory_description tdes;

	tdes.arm_type = lib::XYZ_EULER_ZYZ;
	tdes.interpolation_node_no = 1;
	tdes.internode_step_no = motion_time;
	tdes.value_in_step_no = tdes.internode_step_no - 2;
	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	tdes.coordinate_delta[0] = 0.0; // przyrost wspolrzednej X
	tdes.coordinate_delta[1] = 0.0;// przyrost wspolrzednej Y
	tdes.coordinate_delta[2] = 0.0; // przyrost wspolrzednej Z
	tdes.coordinate_delta[3] = 0.0; // przyrost wspolrzednej FI
	tdes.coordinate_delta[4] = 0.0; // przyrost wspolrzednej TETA
	tdes.coordinate_delta[5] = 0.0; // przyrost wspolrzednej PSI
	//	tdes.coordinate_delta[6] = 0.0;   // przyrost wspolrzednej PSI
	tdes.coordinate_delta[6] = gripper_increment; // przyrost wspolrzednej PSI

	// Generator trajektorii prostoliniowej
	ecp::common::generator::linear lg(_ecp_task, tdes, 1);

	_ecp_task.Move (lg);
}
#endif
// KONSTRUKTORY
fsautomat::fsautomat(lib::configurator &_config) :
	task(_config), gt(NULL), nrg(NULL), rgg(NULL), gag(NULL), rfrg(NULL), tig(NULL), befg(NULL), wmg(NULL),
			go_st(NULL)
{
	// the robot is choose dependendant on the section of configuration file sent as argv[4]
	if (config.section_name == lib::irp6ot_m::ECP_SECTION) {
		ecp_m_robot = new irp6ot_m::robot(*this);
	} else if (config.section_name == lib::irp6p_m::ECP_SECTION) {
		ecp_m_robot = new irp6p_m::robot(*this);
	} else {
		// TODO: throw, robot unsupported
		return;
	}

	const std::string whichECP = lib::toString(ecp_m_robot->robot_name);

	std::string filePath(mrrocpp_network_path);
	std::string fileName = config.value <std::string> ("xml_file", "[xml_settings]");
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
				for (xmlNode *child_node = cur_node->children->children; child_node != NULL; child_node
						= child_node->next) {
					if (child_node->type == XML_ELEMENT_NODE
							&& !xmlStrcmp(child_node->name, (const xmlChar *) "base/ecp")) {
						xmlChar * robot = xmlGetProp(child_node, (const xmlChar *) "name");
						if (robot && !xmlStrcmp(robot, (const xmlChar *) whichECP.c_str())) {
							for (; child_node->children; child_node->children = child_node->children->next) {
								if (child_node->children->type == XML_ELEMENT_NODE) {
									if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "ecp_gen_t")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											;
										gt = new common::generator::transparent(*this);
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "ecp_tff_nose_run_gen")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											nrg = new common::generator::tff_nose_run(*this, atoi((char *) argument));
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "ecp_tff_nose_run_st")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) "")) {
											sub_task::tff_nose_run* ecpst;
											ecpst = new sub_task::tff_nose_run(*this);
											subtask_m[ecp_mp::sub_task::ECP_ST_TFF_NOSE_RUN] = ecpst;
										}
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "ecp_tff_rubik_grab_gen")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											rgg = new common::generator::tff_rubik_grab(*this, atoi((char *) argument));
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "ecp_tff_gripper_approach_gen")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											gag
													= new common::generator::tff_gripper_approach(*this, atoi((char *) argument));
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "ecp_tff_rubik_face_rotate_gen")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											rfrg
													= new common::generator::tff_rubik_face_rotate(*this, atoi((char *) argument));
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "ecp_teach_in_gen")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											;
										tig = new common::generator::teach_in(*this);
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "bias_edp_force_gen")) {

										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											;
										befg = new common::generator::bias_edp_force(*this);
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "bias_edp_force_st")) {

										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											;
										{
											sub_task::sub_task* ecpst;
											ecpst = new sub_task::bias_edp_force(*this);
											subtask_m[ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE] = ecpst;
										}
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "ecp_smooth_gen")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
										//	sg = new common::generator::smooth(*this, (bool) atoi((char *) argument));
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "weight_measure_gen")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											wmg = new common::generator::weight_measure(*this, atoi((char *) argument));
										xmlFree(argument);
									} else if (!xmlStrcmp(child_node->children->name, (const xmlChar *) "gripper_opening")) {
										xmlChar *argument = xmlNodeGetContent(child_node->children);
										if (argument && xmlStrcmp(argument, (const xmlChar *) ""))
											;
										go_st = new common::sub_task::gripper_opening(*this);
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

void fsautomat::main_task_algorithm(void)
{

	std::string fileName = config.value <std::string> ("xml_file", "[xml_settings]");
	int trjConf = config.value <int> ("trajectory_from_xml", "[xml_settings]");
	int ecpLevel = config.value <int> ("trajectory_on_ecp_level", "[xml_settings]");

	if (trjConf && ecpLevel) {
		trjMap = loadTrajectories(fileName.c_str(), ecp_m_robot->robot_name);
		printf("Lista %s zawiera: %d elementow\n", lib::toString(ecp_m_robot->robot_name).c_str(), trjMap->size());
	}

	for (;;) {

		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("Order received");

		subtasks_conditional_execution();

		if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TEACH_IN) {
			std::string path(mrrocpp_network_path);
			path += mp_command.ecp_next_state.mp_2_ecp_next_state_string;
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
					//sg->load_trajectory_from_xml((*trjMap)[mp_command.ecp_next_state.mp_2_ecp_next_state_string]);
				} else {
					std::string path(mrrocpp_network_path);
					path += fileName;
					//sg->load_trajectory_from_xml(path.c_str(), mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				}
			}//if
			else //moj przypadekl -> z pliku
			{
				std::string path(mrrocpp_network_path);
				path += mp_command.ecp_next_state.mp_2_ecp_next_state_string;
				//	sg->get_type_for_smooth_xml(path.c_str());
				//
				//	sg->get_type_for_smooth_xml2(path.c_str(), mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				//sg->load_file_with_path(path.c_str());
			}//else
			//sg->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_WEIGHT_MEASURE) {
			wmg->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TRANSPARENT) {
			gt->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN) {
			nrg->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE) {
			befg->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_RUBIK_GRAB) {
			double gen_args[4];
			int size = lib::setValuesInArray(gen_args, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
			if (size > 3)
				rgg->configure(gen_args[0], gen_args[1], (unsigned int) gen_args[2], (bool) gen_args[3]);
			else
				rgg->configure(gen_args[0], gen_args[1], (unsigned int) gen_args[2]);
			rgg->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_RUBIK_FACE_ROTATE) {
			double gen_args[1];
			lib::setValuesInArray(gen_args, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
			rfrg->configure(gen_args[0]);
			rfrg->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH) {
			double gen_args[2];
			lib::setValuesInArray(gen_args, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
			gag->configure(gen_args[0], (unsigned int) gen_args[1]);
			gag->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::sub_task::ECP_ST_GRIPPER_OPENING) {
			double gen_args[2];
			lib::setValuesInArray(gen_args, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
			go_st->configure(gen_args[0], (int) gen_args[1]);
			go_st->execute();
		}

		ecp_termination_notice();
	} //end for
}

task* return_created_ecp_task(lib::configurator &_config)
{
	return new fsautomat(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
