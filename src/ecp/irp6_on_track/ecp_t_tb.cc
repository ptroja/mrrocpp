#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_tb.h"
//own libraries
//Constructors
ecp_t_tb_irp6ot::ecp_t_tb_irp6ot(configurator &_config): ecp_task(_config){
};

ecp_t_tb_irp6ot::~ecp_t_tb_irp6ot(){
};

// methods for ECP template to redefine in concrete classes
void ecp_t_tb_irp6ot::task_initialization(void){
	ecp_m_robot=new ecp_irp6_on_track_robot(*this);
	sr_ecp_msg->message("ECP loaded tb");
};

void ecp_t_tb_irp6ot::main_task_algorithm(void){
	sr_ecp_msg->message("ECP tb");
	ecp_wait_for_start();
	sr_ecp_msg->message("works");
	ecp_wait_for_stop();
};

ecp_task* return_created_ecp_task(configurator &_config){
	return new ecp_t_tb_irp6ot(_config);
}
