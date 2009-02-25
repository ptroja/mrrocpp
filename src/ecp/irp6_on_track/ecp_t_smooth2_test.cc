#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_smooth2_test.h"

//Constructors
ecp_t_smooth2_test::ecp_t_smooth2_test(configurator &_config): ecp_task(_config){
  smoothgen2 = NULL;
};
//Desctructor
ecp_t_smooth2_test::~ecp_t_smooth2_test(){

};

//methods for ECP template to redefine in concrete classes
void ecp_t_smooth2_test::task_initialization(void) {

	ecp_m_robot = new ecp_irp6_on_track_robot(*this);
	smoothgen2 = new ecp_smooth2_generator(*this, true);
	sr_ecp_msg->message("ECP loaded smooth2_test");
};

void ecp_t_smooth2_test::main_task_algorithm(void ){
	sr_ecp_msg->message("ECP smooth2_test ready");
	ecp_wait_for_start();
  
	smoothgen2->set_absolute();

	if (smoothgen2->load_file_with_path("/net/home-host/mnt/mrroc/trj/box_euler2.trj")) {
    smoothgen2->Move();
  };
  
	//printf("wielkosc listy: %d\n", smoothgen2->pose_list_length());
	
	  //smoothgen2->Move();
	  //sr_ecp_msg->message("jest git");
	//smoothgen2->reset();
	
	ecp_termination_notice();
	ecp_wait_for_stop();
};

ecp_task* return_created_ecp_task(configurator &_config){
	return new ecp_t_smooth2_test(_config);
}
