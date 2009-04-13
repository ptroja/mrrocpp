#include <stdio.h>

#include "lib/srlib.h"
#include "ecp/conveyor/ecp_local.h"
#include "ecp/conveyor/ecp_t_legobrick_conv.h"
//#include "ecp/conveyor/ecp_g_legobrick.h"
#include "ecp/conveyor/ecp_g_visconv.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {
namespace task {


// KONSTRUKTORY
ecp_task_conveyor_lego_brick::ecp_task_conveyor_lego_brick(configurator &_config) : ecp_task(_config)
{
	absolute_position = 0.0;
}

ecp_task_conveyor_lego_brick::~ecp_task_conveyor_lego_brick()
{}

// methods for ECP template to redefine in concrete classes
void ecp_task_conveyor_lego_brick::task_initialization(void)
{
	ecp_m_robot = new ecp_conveyor_robot (*this);

	sr_ecp_msg->message("ECP loaded");

}


void ecp_task_conveyor_lego_brick::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP lego brick - wcisnij start");
	ecp_wait_for_start();
	generator::conveyor_incremental_move sinaaa(*this, 100);
	//ecp_smooth_generator gen(*this, true, true);
	//ecp_smooth_generator gen2(*this, true, true);
	//gen.flush_pose_list();
	//gen2.flush_pose_list();
	
	POSE_SPECIFICATION ps;
	ps = JOINT;

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

	//gen.create_pose_list_head(ps, vp, vk, v, a, coordinates);
	//gen2.create_pose_list_head(ps, vp, vk, v, a, coordinates2);
	//ysg.sensor_m = sensor_m;
	

	for(;;) { // Wewnetrzna petla nieskonczona

		for(;;) {
			sr_ecp_msg->message("Ruch sin");
			sinaaa.Move();
			//gen.Move();
			//gen2.Move();
		}

		// Oczekiwanie na STOP
		//printf("przed wait for stop\n");
		ecp_wait_for_stop ();
		break;
	} // koniec: for(;;) wewnetrznej
}

}
} // namespace conveyor

namespace common {
namespace task {

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new conveyor::task::ecp_task_conveyor_lego_brick(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

