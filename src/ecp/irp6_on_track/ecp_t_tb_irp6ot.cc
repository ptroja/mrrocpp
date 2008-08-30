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
	sr_ecp_msg->message("ECP loaded tb");
};

void ecp_t_tb_irp6ot::main_task_algorithm(void){
	//char *path="/net/home-host/home/mrrocpp/trj/draughts/pawn_moving.trj";
	//file with trajectory to smooth generator
	char *path="/net/lenin/mnt/trj/draughts/pawn_moving.trj";
	
	sr_ecp_msg->message("ECP tb.... ready");
	ecp_wait_for_start();
	
	sgen->load_file_with_path(path);
	sgen->Move();
	
	befgen->Move();
	
	//configuration of gripper approach configure(speed, time_period)
	gagen->configure(0.1,2);
	gagen->Move();
	
	ecp_termination_notice();
	ecp_wait_for_stop();
};

ecp_task* return_created_ecp_task(configurator &_config){
	return new ecp_t_tb_irp6ot(_config);
}
