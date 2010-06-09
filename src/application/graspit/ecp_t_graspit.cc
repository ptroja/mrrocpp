#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp_t_graspit.h"
#include "ecp_mp_tr_graspit.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//Constructors
Graspit::Graspit(lib::configurator &_config): task(_config){

    if (config.section_name == ECP_IRP6_ON_TRACK_SECTION)
    {
        ecp_m_robot = new irp6ot::robot (*this);
    }
    else if (config.section_name == ECP_IRP6_POSTUMENT_SECTION)
    {
        ecp_m_robot = new irp6p::robot (*this);
    }

	smoothgen2 = new common::generator::smooth(*this, true);
	smoothgen2->sensor_m = sensor_m;
	trgraspit = new ecp_mp::transmitter::TRGraspit(ecp_mp::transmitter::TRANSMITTER_GRASPIT,"[transmitter_graspit]",*this);

	sr_ecp_msg->message("ECP loaded graspit");
};

void Graspit::main_task_algorithm(void ){

	sr_ecp_msg->message("ECP graspit ready");

	int port=config.value<int>("graspit_port","[transmitter_graspit]");
	std::string node_name=config.value<std::string>("graspit_node_name","[transmitter_graspit]");

	trgraspit->TRconnect(node_name.c_str(), port);
	trgraspit->t_read();
	trgraspit->TRdisconnect();

	//mrroc++ kinematics,
	//1 affects 2
	//2 affects 3
	trgraspit->from_va.grasp_joint[2] += trgraspit->from_va.grasp_joint[1];
	trgraspit->from_va.grasp_joint[3] += trgraspit->from_va.grasp_joint[2];

	//synchro
	trgraspit->from_va.grasp_joint[0] ;
	trgraspit->from_va.grasp_joint[1] -= 1.542;
	trgraspit->from_va.grasp_joint[2] ;
	trgraspit->from_va.grasp_joint[3] ;
	trgraspit->from_va.grasp_joint[4] += 4.712;
	trgraspit->from_va.grasp_joint[5] -= 2.738;
	trgraspit->from_va.grasp_joint[6] = 0.074;

	cout << trgraspit->from_va.grasp_joint[0] << "\n";
	cout << trgraspit->from_va.grasp_joint[1] << "\n";
	cout << trgraspit->from_va.grasp_joint[2] << "\n";
	cout << trgraspit->from_va.grasp_joint[3] << "\n";
	cout << trgraspit->from_va.grasp_joint[4] << "\n";
	cout << trgraspit->from_va.grasp_joint[5] << "\n";
	cout << trgraspit->from_va.grasp_joint[6] << "\n";

	smoothgen2->set_absolute();
	smoothgen2->load_coordinates(lib::ECP_JOINT,
								trgraspit->from_va.grasp_joint[0],
								trgraspit->from_va.grasp_joint[1],
								trgraspit->from_va.grasp_joint[2],
								trgraspit->from_va.grasp_joint[3],
								trgraspit->from_va.grasp_joint[4],
								trgraspit->from_va.grasp_joint[5],
								trgraspit->from_va.grasp_joint[6],
								0.0, true);
	smoothgen2->Move();
	smoothgen2->reset();

	ecp_termination_notice();
};

} // namespace task
} // namespace common

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new common::task::Graspit(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


