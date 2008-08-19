//#include "common/typedefs.h"
//#include "common/impconst.h"
//#include "common/com_buf.h"

//#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_tb_irp6ot.h"
#include <stdio.h>
//#include "ecp/common/ecp_generator_t.h"

//own libraries

//Constructors
ecp_t_tb_irp6ot::ecp_t_tb_irp6ot(configurator &_config): ecp_task(_config){
	sgen=NULL;
};

ecp_t_tb_irp6ot::~ecp_t_tb_irp6ot(){
};

// methods for ECP template to redefine in concrete classes
void ecp_t_tb_irp6ot::task_initialization(void){
	ecp_m_robot=new ecp_irp6_on_track_robot(*this);
	sgen=new ecp_smooth_generator(*this, true);
	sr_ecp_msg->message("ECP loaded tb");
};

void ecp_t_tb_irp6ot::main_task_algorithm(void){
	char *path="/net/home-host/home/mrrocpp/trj/draughts/pawn_moving.trj";
	
	sr_ecp_msg->message("ECP tb.... ready");
	ecp_wait_for_start();
	//sr_ecp_msg->message("works");
	sgen->load_file_with_path(path);
	sgen->Move();
	ecp_termination_notice();
	ecp_wait_for_stop();
};

ecp_task* return_created_ecp_task(configurator &_config){
	return new ecp_t_tb_irp6ot(_config);
}
