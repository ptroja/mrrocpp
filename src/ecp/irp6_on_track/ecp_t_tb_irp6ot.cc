//#include "common/typedefs.h"
//#include "common/impconst.h"
//#include "common/com_buf.h"

//#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_tb_irp6ot.h"
#include <stdio.h>


//Constructors
ecp_t_tb_irp6ot::ecp_t_tb_irp6ot(configurator &_config): ecp_task(_config){
	sgen=NULL;
	befgen=NULL;
	gagen=NULL;
	lgen=NULL;
};

ecp_t_tb_irp6ot::~ecp_t_tb_irp6ot(){
};

// methods for ECP template to redefine in concrete classes
void ecp_t_tb_irp6ot::task_initialization(void){
	//initialization of robot
	ecp_m_robot=new ecp_irp6_on_track_robot(*this);
	sgen=new ecp_smooth_generator(*this, true);
	befgen=new bias_edp_force_generator(*this);
	//gripper approach constructor (task&, no_of_steps)
	gagen=new ecp_tff_gripper_approach_generator (*this, 8);
	go_st = new ecp_sub_task_gripper_opening(*this);
	sr_ecp_msg->message("ECP loaded tb");
};

void ecp_t_tb_irp6ot::main_task_algorithm(void){
	//char *path="/net/home-host/home/mrrocpp/trj/draughts/1_pawn_moving.trj";
	//file with trajectory to smooth generator
	char *path="/net/lenin/home/mrrocpp/trj/draughts/1_pawn_moving.trj";
	
	sr_ecp_msg->message("ECP tb.... ready");
	ecp_wait_for_start();
	
	sgen->load_file_with_path(path);
	sgen->Move();
	
	befgen->Move();
	
	//configuration of gripper approach configure(speed, time_period)
	gagen->configure(0.01,500);
	gagen->Move();
	
	//configuration and initalization of linear generator
	init_tdes(XYZ_ANGLE_AXIS,500);
	set_tdes(0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.0);	//podniesienie o 2mm

	// Generator trajektorii prostoliniowej
	lgen=new ecp_linear_generator(*this,tdes,1);
	//lgen->Move();
	delete lgen;
	
	set_tdes(0,0,0,0,0,0,0.01);	//gripper closing
	lgen=new ecp_linear_generator(*this,tdes,1);
	//lgen->Move();
	delete lgen;	
	
	set_tdes(0,0,0.02,0,0,0,0);	//podniesienie o 2cm
	lgen=new ecp_linear_generator(*this,tdes,1);
	//lgen->Move();
	delete lgen;
	
	char *path1="/net/lenin/home/mrrocpp/trj/draughts/2_pawn_moving.trj";
	//char *path1="/net/home-host/home/mrrocpp/trj/draughts/2_pawn_moving.trj";
	sr_ecp_msg->message(path1);
	sgen->load_file_with_path(path1);
	//sgen->Move();
	
	ecp_termination_notice();
	ecp_wait_for_stop();
};

ecp_task* return_created_ecp_task(configurator &_config){
	return new ecp_t_tb_irp6ot(_config);
}

void ecp_t_tb_irp6ot::set_tdes(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6){
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
void ecp_t_tb_irp6ot::init_tdes(POSE_SPECIFICATION pspec, int internode_no){
	tdes.arm_type=pspec;
	tdes.interpolation_node_no=1;
	tdes.internode_step_no=internode_no;	//motion time
	tdes.value_in_step_no=internode_no-2;			//motion time-2 ??
} 
