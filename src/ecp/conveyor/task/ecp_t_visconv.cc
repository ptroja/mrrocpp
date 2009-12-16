#include <stdio.h>

#include "lib/srlib.h"
#include "ecp/conveyor/ecp_r_conv.h"
#include "ecp/conveyor/task/ecp_t_legobrick_conv.h"
//#include "ecp/conveyor/generator/ecp_g_legobrick.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {
namespace task {

// KONSTRUKTORY
lego_brick::lego_brick(lib::configurator &_config) : task(_config)
{
	absolute_position = 0.0;

	ecp_m_robot = new robot (*this);

	sr_ecp_msg->message("ECP loaded");
}

void lego_brick::main_task_algorithm(void)
{
	//conveyor_incremental_move ysg(*this, 100);
	common::generator::smooth gen(*this, true, true);
	common::generator::smooth gen2(*this, true, true);

	lib::POSE_SPECIFICATION ps = lib::JOINT;

	double coordinates[MAX_SERVOS_NR];
	double coordinates2[MAX_SERVOS_NR];
	double vp[MAX_SERVOS_NR];
	double vk[MAX_SERVOS_NR];
	double v[MAX_SERVOS_NR];
	double a[MAX_SERVOS_NR];

	//absolute_position-= 0.25;
	coordinates[0] = absolute_position-0.25;
	coordinates2[0] = absolute_position;

	vp[0] = 0.0;
	vk[0] = 0.0;
	v[0] = 0.04;
	a[0] = 0.002;
	for(int i = 1; i < MAX_SERVOS_NR; ++i){
		vp[i] = 0.0;
		vk[i] = 0.0;
		v[i] = 0.0;
		a[i] = 0.0;
		coordinates[i] = 0.0;
		coordinates2[i] = 0.0;
	}

	gen.insert_pose_list_element(ps, vp, vk, v, a, coordinates);
	gen2.insert_pose_list_element(ps, vp, vk, v, a, coordinates2);

	for(;;) {
		sr_ecp_msg->message("Ruch");

		gen.Move();
		gen2.Move();
	}
}

}
} // namespace conveyor

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new conveyor::task::lego_brick(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

