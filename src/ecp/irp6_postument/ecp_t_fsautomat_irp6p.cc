// ------------------------------------------------------------------------
//   ecp_t_fsautomat_irp6p.cc - zadanie przelewania, ECP dla IRP6_POSTUMENT
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2008
// ------------------------------------------------------------------------


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <map>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_fsautomat.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/common/ecp_t_fsautomat.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp/irp6_postument/ecp_t_fsautomat_irp6p.h"

#include "ecp_mp/Trajectory.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

// KONSTRUKTORY
fsautomat::fsautomat(lib::configurator &_config) : task(_config)
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

void fsautomat::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_postument_robot (*this);

	const char * whichECP = "ROBOT_IRP6_POSTUMENT";

	int conArg;
	xmlNode *cur_node, *child_node;
	xmlChar *robot;
	xmlChar *stateType, *argument;

	std::string filePath(mrrocpp_network_path);
	std::string fileName = config.return_string_value("xml_file", "[xml_settings]");
	filePath += fileName;

	// open xml document
	xmlDocPtr doc;
	doc = xmlParseFile(filePath.c_str());
	if(doc == NULL)
	{
		fprintf(stderr, "ERROR in ecp initialization: could not parse file: %s\n",fileName.c_str());
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
									gt = new common::generator::transparent(*this);
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_tff_nose_run_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""))
										nrg = new common::generator::tff_nose_run(*this, atoi((char *)argument));
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_tff_rubik_grab_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""))
										rgg = new common::generator::tff_rubik_grab(*this, atoi((char *)argument));
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_tff_gripper_approach_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""))
										gag = new common::generator::tff_gripper_approach(*this, atoi((char *)argument));
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_tff_rubik_face_rotate_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""))
										rfrg = new common::generator::tff_rubik_face_rotate(*this, atoi((char *)argument));
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
									befg = new common::generator::bias_edp_force(*this);
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_tool_change_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""))
										tcg = new common::generator::tool_change(*this, atoi((char *)argument));
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"ecp_smooth_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""))
										sg = new common::generator::smooth(*this, (bool)atoi((char *)argument));
									xmlFree(argument);
								}
								if(child_node->children->type == XML_ELEMENT_NODE &&
										!xmlStrcmp(child_node->children->name, (const xmlChar *)"weight_meassure_gen"))
								{
									argument = xmlNodeGetContent(child_node->children);
									if(argument && xmlStrcmp(argument, (const xmlChar *)""))
										wmg = new common::generator::weight_meassure(*this, atoi((char *)argument));
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
};

void fsautomat::main_task_algorithm(void)
{
	std::string fileName = config.return_string_value("xml_file", "[xml_settings]");
	int trjConf = config.return_int_value("trajectory_from_xml", "[xml_settings]");
	int ecpLevel = config.return_int_value("trajectory_on_ecp_level", "[xml_settings]");

	if(trjConf && ecpLevel)
	{
		trjMap = loadTrajectories(fileName.c_str(), lib::ROBOT_IRP6_POSTUMENT);
		printf("Lista ROBOT_IRP6_POSTUMENT zawiera: %d elementow\n", trjMap->size());
	}

	for(;;) {
		std::string path1;

		sr_ecp_msg->message("Waiting for MP order");

		get_next_state ();

		sr_ecp_msg->message("Order received");

		switch ( (ecp_mp::task::STATE_MACHINE_ECP_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state)
		{
			case ecp_mp::task::ECP_GEN_TEACH_IN:

				path1 = mrrocpp_network_path;
				path1 += mp_command.ecp_next_state.mp_2_ecp_next_state_string;
				tig->flush_pose_list();
				tig->load_file_with_path (path1.c_str());
				tig->initiate_pose_list();
				tig->Move();
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
						path1 = mrrocpp_network_path;
						path1 += fileName;
						sg->load_trajectory_from_xml(path1.c_str(), mp_command.ecp_next_state.mp_2_ecp_next_state_string);
					}
				}
				else
				{
					path1 = mrrocpp_network_path;
					path1 += mp_command.ecp_next_state.mp_2_ecp_next_state_string;
					sg->load_file_with_path (path1.c_str());
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
			{
				double gen_args[4];
				int size = ecp_mp::common::Trajectory::setValuesInArray(gen_args, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				if(size > 3)
					rgg->configure(gen_args[0], gen_args[1], (int)gen_args[2], (bool)gen_args[3]);
				else
					rgg->configure(gen_args[0], gen_args[1], (int)gen_args[2]);
				rgg->Move();
				break;
			}
			case ecp_mp::task::ECP_GEN_TFF_RUBIK_FACE_ROTATE:
			{
				double gen_args[1];
				ecp_mp::common::Trajectory::setValuesInArray(gen_args, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				rfrg->configure(gen_args[0]);
				rfrg->Move();
				break;
			}
			case ecp_mp::task::ECP_GEN_TFF_GRIPPER_APPROACH:
			{
				double gen_args[2];
				ecp_mp::common::Trajectory::setValuesInArray(gen_args, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				gag->configure(gen_args[0] , (int)gen_args[1]);
				gag->Move();
				break;
			}
			case ecp_mp::task::ECP_TOOL_CHANGE_GENERATOR:
			{
				double gen_args[3];
				ecp_mp::common::Trajectory::setValuesInArray(gen_args, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				tcg->set_tool_parameters(gen_args[0], gen_args[1], gen_args[2]);
				tcg->Move();
				break;
			}
			case ecp_mp::task::RCSC_GRIPPER_OPENING:
			{
				double gen_args[2];
				ecp_mp::common::Trajectory::setValuesInArray(gen_args, mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				go_st->configure(gen_args[0], (int)gen_args[1]);
				go_st->execute();
				break;
			}
			default:
				break;
		} // end switch
		ecp_termination_notice();

	} //end for
}

}
} // namespace irp6p

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6p::task::fsautomat(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

