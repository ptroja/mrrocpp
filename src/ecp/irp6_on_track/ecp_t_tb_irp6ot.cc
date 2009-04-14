//#include "lib/typedefs.h"
//#include "lib/impconst.h"
//#include "lib/com_buf.h"

//#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_tb_irp6ot.h"
#include <stdio.h>
//#define sim

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

//Constructors
tb::tb(lib::configurator &_config): base(_config){
	sgen=NULL;
	befgen=NULL;
	gagen=NULL;
	lgen=NULL;
};

tb::~tb(){
};

// methods for ECP template to redefine in concrete classes
void tb::task_initialization(void){

	sensor_m[lib::SENSOR_CVFRADIA] = new ecp_mp::sensor::cvfradia(lib::SENSOR_CVFRADIA,"[vsp_cvfradia]", *this,	sizeof(lib::sensor_image_t::sensor_union_t::fradia_t));
	sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();

	ecp_m_robot=new ecp_irp6_on_track_robot(*this);				//initialization of robot
	sgen=new common::generator::smooth(*this, true);
	befgen=new common::generator::bias_edp_force(*this);
	gagen=new common::generator::tff_gripper_approach (*this, 8);	//gripper approach constructor (task&, no_of_steps)
	go_st = new common::task::ecp_sub_task_gripper_opening(*this);
	sleepgen=new common::generator::sleep(*this);
	sr_ecp_msg->message("ECP loaded tb");
};

void tb::main_task_algorithm(void){
	//char *path="/net/home-host/home/mrrocpp/trj/draughts/1_pawn_moving.trj";
	//file with trajectory to smooth generator

/*
	double vp[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double vk[MAX_SERVOS_NR]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double v[MAX_SERVOS_NR]={0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 5.0};
	double a[MAX_SERVOS_NR]={0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.5};
	double coordinates[MAX_SERVOS_NR];

	coordinates[0]=0;
	coordinates[1]=-30;
	coordinates[2]=0;
	coordinates[3]=0;
	coordinates[4]=0;
	coordinates[5]=0;
	coordinates[6]=0;
	coordinates[7]=0;
	*/


	//test part
	/*
	//sgen=new ecp_smooth_generator(*this, true);
	sgen->set_absolute();
	sgen->load_coordinates(lib::MOTOR,vp,vk,v,a,coordinates);
	sgen->Move();
	sgen->reset();
	//delete sgen;

	//sgen=new ecp_smooth_generator(*this, true);
	sgen->set_relative();
	coordinates[0]=0;
	coordinates[1]=-20;
	coordinates[2]=0;
	coordinates[3]=0;
	coordinates[4]=0;
	coordinates[5]=0;
	coordinates[6]=0;
	coordinates[7]=0;
	sgen->load_coordinates(lib::MOTOR,vp,vk,v,a,coordinates);
		coordinates[0]=0;
	coordinates[1]=0;
	coordinates[2]=0;
	coordinates[3]=10;
	coordinates[4]=0;
	coordinates[5]=0;
	coordinates[6]=0;
	coordinates[7]=0;
	sgen->load_coordinates(lib::MOTOR,vp,vk,v,a,coordinates);
		coordinates[0]=0;
	coordinates[1]=0;
	coordinates[2]=0;
	coordinates[3]=10;
	coordinates[4]=0;
	coordinates[5]=0;
	coordinates[6]=0;
	coordinates[7]=0;
	sgen->load_coordinates(lib::MOTOR,vp,vk,v,a,coordinates);
		coordinates[0]=0;
	coordinates[1]=0;
	coordinates[2]=0;
	coordinates[3]=10;
	coordinates[4]=0;
	coordinates[5]=0;
	coordinates[6]=0;
	coordinates[7]=0;
	sgen->load_coordinates(lib::MOTOR,vp,vk,v,a,coordinates);
	sgen->Move();
	delete sgen;
	*/
	/*
	//sgen=new ecp_smooth_generator(*this, true);
	sgen->set_absolute();
	sgen->load_coordinates(lib::MOTOR,0,-30,0,0,0,0,0,0);
	sgen->Move();
	sgen->reset();
	//delete sgen;

	//sgen=new ecp_smooth_generator(*this, true);
	sgen->set_relative();
	sgen->load_coordinates(lib::MOTOR,0,-20,0,0,0,0,0,0);
	sgen->load_coordinates(lib::MOTOR,0,0,0,10,0,0,0,0);
	sgen->load_coordinates(lib::MOTOR,0,0,0,10,0,0,0,0);
	sgen->load_coordinates(lib::MOTOR,0,0,0,10,0,0,0,0);
	sgen->Move();
	delete sgen;


	sgen=new ecp_smooth_generator(*this, true);
	sgen->set_absolute();
	sgen->load_coordinates(lib::MOTOR,0,0,0,0,0,0,0,0);
	sgen->Move();
	delete sgen;
*/
	//sgen=new ecp_smooth_generator(*this, true);
	#ifdef sim
		sgen->load_file_with_path("/net/home-host/home/mrrocpp/trj/draughts/test2.trj");
	#else
		sgen->load_file_with_path("/net/lenin/home/mrrocpp/trj/draughts/test2.trj");
	#endif

	sgen->Move();
	//delete sgen;
	sleepgen->init_time(10);
	sleepgen->Move();
	//sgen=new ecp_smooth_generator(*this, true);
	sgen->set_relative();
	#ifdef sim
		sgen->load_file_with_path("/net/home-host/home/mrrocpp/trj/draughts/test1.trj");
	#else
		sgen->load_file_with_path("/net/lenin/home/mrrocpp/trj/draughts/test1.trj");
	#endif
	sgen->Move();
	//delete sgen;
	/*
	sgen->set_absolute();
	sgen->reset();
	sgen->load_coordinates(lib::MOTOR,0,0,0,0,0,0,0,0);
	sgen->load_coordinates(lib::MOTOR,0,-45,0,9,0,0,0,0);
	sgen->Move();
	sgen->reset();
	sgen->set_relative();
	sgen->load_coordinates(lib::MOTOR,0,0,0,10,0,0,0,0);
	sgen->load_coordinates(lib::MOTOR,0,0,0,-10,0,0,0,0);
	sgen->Move();

	sgen->set_absolute();
	#ifdef sim
		sgen->load_file_with_path("/net/home-host/home/mrrocpp/trj/draughts/test2.trj");
	#else
		sgen->load_file_with_path("/net/lenin/home/mrrocpp/trj/draughts/test2.trj");
	#endif
	sgen->Move();
	sgen->reset();

	sgen->load_coordinates(lib::MOTOR,0,0,0,0,0,0,0,0);
	sgen->Move();
	*/
	//sgen=new ecp_smooth_generator(*this, true);


	/*
	//moving pawn part
	char *path="/net/lenin/home/mrrocpp/trj/draughts/1_pawn_moving.trj";
	sgen->load_file_with_path(path);
	sgen->Move();

	befgen->Move();

	//configuration of gripper approach configure(speed, time_period)
	gagen->configure(0.01,500);
	gagen->Move();

	sgen->set_relative();
	sgen->reset();
	sgen->load_coordinates(lib::MOTOR,0,0,0,-1,0,0,0,0);
	sgen->load_coordinates(lib::MOTOR,0,0,0,0,0,0,0,1050);
	sgen->Move();
	sgen->reset();
	printf("ala");
	sgen->load_coordinates(lib::MOTOR,0,0,0,-5,0,0,0,0);
	sgen->Move();*/
	/*
	//configuration and initalization of linear generator
	init_tdes(lib::XYZ_ANGLE_AXIS,500);
	set_tdes(0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.0);	//podniesienie o 2mm

	// Generator trajektorii prostoliniowej
	lgen=new ecp_linear_generator(*this,tdes,1);
	lgen->Move();
	delete lgen;

	set_tdes(0,0,0,0,0,0,-0.020);	//gripper closing
	lgen=new ecp_linear_generator(*this,tdes,1);
	lgen->Move();
	delete lgen;

	set_tdes(0,0,0.02,0,0,0,0);	//podniesienie o 2cm
	lgen=new ecp_linear_generator(*this,tdes,1);
	lgen->Move();
	delete lgen;
	*/
	/*
	char *path1="/net/lenin/home/mrrocpp/trj/draughts/2_pawn_moving.trj";
	//char *path1="/net/home-host/home/mrrocpp/trj/draughts/2_pawn_moving.trj";
	sgen->load_file_with_path(path1);
	sgen->Move();
	*/

	ecp_termination_notice();
};


void tb::set_tdes(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6){
	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	tdes.coordinate_delta[0] = cor0; // przyrost wspolrzednej X
	tdes.coordinate_delta[1] = cor1;// przyrost wspolrzednej Y
	tdes.coordinate_delta[2] = cor2; // przyrost wspolrzednej Z
	tdes.coordinate_delta[3] = cor3; // przyrost wspolrzednej FI
	tdes.coordinate_delta[4] = cor4; // przyrost wspolrzednej TETA
	tdes.coordinate_delta[5] = cor5; // przyrost wspolrzednej PSI
	tdes.coordinate_delta[6] = cor6; // przyrost wspolrzednej PSI
}

//inicjacja struktury tdes - trajectory description
void tb::init_tdes(lib::POSE_SPECIFICATION pspec, int internode_no){
	tdes.arm_type=pspec;
	tdes.interpolation_node_no=1;
	tdes.internode_step_no=internode_no;	//motion time
	tdes.value_in_step_no=internode_no-2;			//motion time-2 ??
}

}
} // namespace irp6ot

namespace common {
namespace task {

base* return_created_ecp_task(lib::configurator &_config){
	return new irp6ot::task::tb(_config);
	
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

