// ------------------------------------------------------------------------
//   ecp_t_fsautomat_irp6ot.cc - automat skonczony , ECP dla IRP6_ON_TRACK
//
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
//
// Ostatnia modyfikacja: 2008
// ------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "mp/Trajectory.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_fsautomat.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp/common/ecp_t_fsautomat.h"
#include "ecp/irp6_on_track/ecp_t_fsautomat_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

// KONSTRUKTORY
ecp_task_fsautomat_irp6ot::ecp_task_fsautomat_irp6ot(configurator &_config) : ecp_task(_config)
{
	sg = NULL;
	tcg = NULL;
	gt = NULL;
	nrg = NULL;
	rgg = NULL;
	gag = NULL;
	rfrg = NULL;
	tig = NULL;
	befg = NULL;
	wmg = NULL;

	go_st = NULL;
}

// methods for ECP template to redefine in concrete classes
void ecp_task_fsautomat_irp6ot::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
	const char * whichECP = "ROBOT_IRP6_ON_TRACK";

	int size, conArg;
	char *filePath;
	char *fileName = config.return_string_value("xml_file", "[xml_settings]");
	xmlNode *cur_node, *child_node;
	xmlChar *robot;
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
	            if ( child_node->type == XML_ELEMENT_NODE  && !xmlStrcmp(child_node->name, (const xmlChar *)"ecp") )
	            {
						robot = xmlGetProp(child_node, (const xmlChar *)"name");
	               if(robot && !xmlStrcmp(robot, (const xmlChar *) whichECP))
						{
							for(;child_node->children; child_node->children = child_node->children->next)
							{
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_gen_t"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""));
									gt = new common::ecp_generator_t(*this);
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_tff_nose_run_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""))
										nrg = new common::ecp_tff_nose_run_generator(*this, atoi((char *)argument));
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_tff_rubik_grab_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""))
										rgg = new common::ecp_tff_rubik_grab_generator(*this, atoi((char *)argument));
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_tff_gripper_approach_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""))
										gag = new common::ecp_tff_gripper_approach_generator(*this, atoi((char *)argument));
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_tff_rubik_face_rotate_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""))
										rfrg = new common::ecp_tff_rubik_face_rotate_generator(*this, atoi((char *)argument));
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_teach_in_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""));
									tig = new common::ecp_teach_in_generator(*this);
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"bias_edp_force_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""));
									befg = new common::bias_edp_force_generator(*this);
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_tool_change_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""))
										tcg = new common::ecp_tool_change_generator(*this, atoi((char *)argument));
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_smooth_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""))
										sg = new common::ecp_smooth_generator(*this, (bool)atoi((char *)argument));
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"weight_meassure_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""))
										wmg = new common::weight_meassure_generator(*this, atoi((char *)argument));
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_sub_task_gripper_opening"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""));
									go_st = new common::task::ecp_sub_task_gripper_opening(*this);
									xmlFree(argument);
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

   sr_ecp_msg->message("ECP loaded");
}

void ecp_task_fsautomat_irp6ot::main_task_algorithm(void)
{
	int size;
	char * path1;
	double *gen_args;
	char * fileName = config.return_string_value("xml_file", "[xml_settings]");
	int trjConf = config.return_int_value("trajectory_from_xml", "[xml_settings]");
	int ecpLevel = config.return_int_value("trajectory_on_ecp_level", "[xml_settings]");

	if(trjConf && ecpLevel)
	{
		trjMap = loadTrajectories(fileName, ROBOT_IRP6_ON_TRACK);
		printf("Lista ROBOT_IRP6_ON_TRACK zawiera: %d elementow\n", trjMap->size());
	}

//	for(std::map<char *, Trajectory, ecp_task_fsautomat_irp6ot::str_cmp>::iterator ii = trjMap->begin(); ii != trjMap->end(); ++ii)
//	{
//		printf("Key: #%s#\n", (*ii).first);
//		(*ii).second.showTime();
//	}

//	printf("ontrack z konfiguracji: %s\n", config.return_string_value("node_name", "[mp]"));
//	printf("Elementow approach_1: %d\n", trjMap->count("depart_1"));
//	(*trjMap)["approach_1"].showTime();

	for(;;)
	{
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state ();

		sr_ecp_msg->message("Order received");

		switch ( (ecp_mp::task::STATE_MACHINE_ECP_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state)
		{
			case ecp_mp::task::ECP_GEN_TEACH_IN:
				size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				path1 = new char[size];
				sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				tig->flush_pose_list();
				//tig->load_file_with_path (path1);
				//tig->initiate_pose_list();
				tig->teach(MOTOR, "asdasdkjasdj");
				if(operator_reaction("Save?"))
					tig->save_file(MOTOR);
				delete[] path1;
				//tig->Move();
				break;
			case ecp_mp::task::ECP_GEN_SMOOTH:
				if(trjConf)
				{
					if(ecpLevel)
					{
						sg->load_trajectory_from_xml((*trjMap)[mp_command.ecp_next_state.mp_2_ecp_next_state_string]);
					}
					else
					{
						size = 1 + strlen(mrrocpp_network_path) + strlen(fileName);
						path1 = new char[size];
						sprintf(path1, "%s%s", mrrocpp_network_path, fileName);
						sg->load_trajectory_from_xml(path1, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
						delete[] path1;
					}
				}
				else
				{
					size = 1 + strlen(mrrocpp_network_path) + strlen(mp_command.ecp_next_state.mp_2_ecp_next_state_string);
					path1 = new char[size];
					sprintf(path1, "%s%s", mrrocpp_network_path, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
					sg->load_file_with_path (path1);
					delete[] path1;
				}
				sg->Move();
				break;
			case ecp_mp::task::ECP_WEIGHT_MEASURE_GENERATOR:
				wmg->Move();
				break;
			case ecp_mp::task::ECP_GEN_TRANSPARENT:
				gt->Move();
				break;
			case ecp_mp::task::ECP_GEN_BIAS_EDP_FORCE:
				befg->Move();
				break;
			case ecp_mp::task::ECP_GEN_TFF_NOSE_RUN:
				nrg->Move();
				break;
			case ecp_mp::task::ECP_GEN_TFF_RUBIK_GRAB:
				gen_args = new double[4];
				size = mp::common::Trajectory::setValuesInArray(gen_args, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				if(size > 3)
					rgg->configure(gen_args[0], gen_args[1], (int)gen_args[2], (bool)gen_args[3]);
				else
					rgg->configure(gen_args[0], gen_args[1], (int)gen_args[2]);
				rgg->Move();
				delete[] gen_args;
				break;
			case ecp_mp::task::ECP_GEN_TFF_RUBIK_FACE_ROTATE:
				gen_args = new double[1];
				mp::common::Trajectory::setValuesInArray(gen_args, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				rfrg->configure(gen_args[0]);
				rfrg->Move();
				delete[] gen_args;
				break;
			case ecp_mp::task::ECP_GEN_TFF_GRIPPER_APPROACH:
				gen_args = new double[2];
				mp::common::Trajectory::setValuesInArray(gen_args, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				gag->configure(gen_args[0] , (int)gen_args[1]);
				gag->Move();
				delete[] gen_args;
				break;
			case ecp_mp::task::ECP_TOOL_CHANGE_GENERATOR:
				gen_args = new double[3];
				mp::common::Trajectory::setValuesInArray(gen_args, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				tcg->set_tool_parameters(gen_args[0], gen_args[1], gen_args[2]);
				tcg->Move();
				delete[] gen_args;
				break;
			case ecp_mp::task::RCSC_GRIPPER_OPENING:
				gen_args = new double[2];
				mp::common::Trajectory::setValuesInArray(gen_args, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				go_st->configure(gen_args[0], (int)gen_args[1]);
				go_st->execute();
				delete[] gen_args;
				break;
			default:
				break;
		}
		ecp_termination_notice();
	} //end for
}

} // namespace irp6ot

namespace common {
namespace task {

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new irp6ot::ecp_task_fsautomat_irp6ot(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

