#include <stdio.h>
#include <string.h>
#include <unistd.h>


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
	//delay(20000);
	smoothgen2 = new ecp_smooth2_generator(*this, true);
	sr_ecp_msg->message("ECP loaded smooth2_test");
};

void ecp_t_smooth2_test::main_task_algorithm(void ){
	//ecp_m_robot = new ecp_irp6_on_track_robot(*this);
	//smoothgen2 = new ecp_smooth2_generator(*this, true);
	//sr_ecp_msg->message("ECP loaded smooth2_test");

	sr_ecp_msg->message("ECP smooth2_test ready");

	smoothgen2->set_absolute();

	if (smoothgen2->load_file_with_path("/mnt/mrroc/workspace/MRROC++/trj/box_euler3.trj")) {
		smoothgen2->Move();
	};

	char size[10];
	double size2 = smoothgen2->pose_list_length();
	sprintf(size,"%f",size2);

	sr_ecp_msg->message(size);

	//printf("wielkosc listy: %d\n", smoothgen2->pose_list_length());
	//fflush();

	  //smoothgen2->Move();
	  //sr_ecp_msg->message("jest git");
	smoothgen2->reset();

	ecp_termination_notice();
};

ecp_task* return_created_ecp_task(configurator &_config){
	return new ecp_t_smooth2_test(_config);
}
