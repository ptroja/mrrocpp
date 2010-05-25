#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp_t_graspit.h"
#include "ecp_mp_tr_graspit.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//Constructors
Graspit::Graspit(lib::configurator &_config): task(_config){

    if (config.section_name == ECP_IRP6OT_M_SECTION)
    {
        ecp_m_robot = new irp6ot_m::robot (*this);
    }
    else if (config.section_name == ECP_IRP6P_M_SECTION)
    {
        ecp_m_robot = new irp6p_m::robot (*this);
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

	double v[8], a[8];
	for (int i=0; i<8; ++i) {
		v[i] = 0.1;
		a[i] = 0.1;
	}

	trgraspit->TRconnect(node_name.c_str(), port);
	trgraspit->t_read();
	trgraspit->TRdisconnect();

	//mrroc++ kinematics,
	//1 affects 2
	//2 affects 3
	trgraspit->from_va.graspit.grasp_joint[2] += trgraspit->from_va.graspit.grasp_joint[1];
	trgraspit->from_va.graspit.grasp_joint[3] += trgraspit->from_va.graspit.grasp_joint[2];

	trgraspit->from_va.graspit.grasp_joint[9] += trgraspit->from_va.graspit.grasp_joint[8];
	trgraspit->from_va.graspit.grasp_joint[10] += trgraspit->from_va.graspit.grasp_joint[9];

	//synchro TODO: move to GraspIt!
	trgraspit->from_va.graspit.grasp_joint[0] ;
	trgraspit->from_va.graspit.grasp_joint[1] -= 1.542;
	trgraspit->from_va.graspit.grasp_joint[2] ;
	trgraspit->from_va.graspit.grasp_joint[3] ;
	trgraspit->from_va.graspit.grasp_joint[4] += 4.712;
	trgraspit->from_va.graspit.grasp_joint[5] ;
	trgraspit->from_va.graspit.grasp_joint[6] = (45.5 - trgraspit->from_va.graspit.grasp_joint[6]) * 2;
	trgraspit->from_va.graspit.grasp_joint[6] /= 1000;

	trgraspit->from_va.graspit.grasp_joint[7] ;
	trgraspit->from_va.graspit.grasp_joint[8] -= 1.542;
	trgraspit->from_va.graspit.grasp_joint[9] ;
	trgraspit->from_va.graspit.grasp_joint[10] ;
	trgraspit->from_va.graspit.grasp_joint[11] += 4.712;
	trgraspit->from_va.graspit.grasp_joint[12] ;
	trgraspit->from_va.graspit.grasp_joint[13] = (45.5 - trgraspit->from_va.graspit.grasp_joint[13]) * 2;
	trgraspit->from_va.graspit.grasp_joint[13] /= 1000;

	smoothgen2->load_coordinates(lib::ECP_JOINT, v, a,
								trgraspit->from_va.graspit.grasp_joint[7],
								trgraspit->from_va.graspit.grasp_joint[8],
								trgraspit->from_va.graspit.grasp_joint[9],
								trgraspit->from_va.graspit.grasp_joint[10],
								trgraspit->from_va.graspit.grasp_joint[11],
								trgraspit->from_va.graspit.grasp_joint[12],
								0.08,
								0.0, false);
	smoothgen2->Move();
	smoothgen2->reset();
	smoothgen2->load_coordinates(lib::ECP_JOINT, v, a,
								trgraspit->from_va.graspit.grasp_joint[0],
								trgraspit->from_va.graspit.grasp_joint[1],
								trgraspit->from_va.graspit.grasp_joint[2],
								trgraspit->from_va.graspit.grasp_joint[3],
								trgraspit->from_va.graspit.grasp_joint[4],
								trgraspit->from_va.graspit.grasp_joint[5],
								0.08,
								0.0, false);
	smoothgen2->Move();
	smoothgen2->reset();
	smoothgen2->load_coordinates(lib::ECP_JOINT, v, a,
								trgraspit->from_va.graspit.grasp_joint[0],
								trgraspit->from_va.graspit.grasp_joint[1],
								trgraspit->from_va.graspit.grasp_joint[2],
								trgraspit->from_va.graspit.grasp_joint[3],
								trgraspit->from_va.graspit.grasp_joint[4],
								trgraspit->from_va.graspit.grasp_joint[5],
								trgraspit->from_va.graspit.grasp_joint[6] > 0.08 ? 0.08 : trgraspit->from_va.graspit.grasp_joint[6] < 0.06 ? 0.06 : trgraspit->from_va.graspit.grasp_joint[6],
								0.0, false);
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


