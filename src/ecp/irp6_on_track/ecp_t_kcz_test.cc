#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/ecp_t_kcz_test.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

//Constructors
kcz_test::kcz_test(lib::configurator &_config): task(_config)
{
	ecp_m_robot = new robot(*this);

	//delay(20000);
	smoothgen2 = new common::generator::smooth2(*this, true);

    if (_config.exists("kwadrat_bok", "[ecp_irp6_on_track]"))
        kw_bok = _config.return_double_value("kwadrat_bok", "[ecp_irp6_on_track]");
    else
    	kw_bok = 0.0;

	sr_ecp_msg->message("ECP loaded kcz_test");
};

void kcz_test::main_task_algorithm(void ) {
	//ecp_m_robot = new ecp_irp6_on_track_robot(*this);
	//smoothgen2 = new ecp_smooth2_generator(*this, true);
	//sr_ecp_msg->message("ECP loaded smooth2_test");

	sr_ecp_msg->message("ECP kcz_test ready");

	smoothgen2->set_absolute();

	//smoothgen2->load_file_with_path("/net/koleszko/mnt/mrroc/trj/smooth2test.trj");
	smoothgen2->load_coordinates(lib::XYZ_EULER_ZYZ, 0.849, -0.298, 	   0.100, 		 -0.004, 1.560, -3.141, 0.074, 0.000, false);
	smoothgen2->load_coordinates(lib::XYZ_EULER_ZYZ, 0.849, -0.298+kw_bok, 0.100, 		 -0.004, 1.560, -3.141, 0.074, 0.000, false);
	smoothgen2->load_coordinates(lib::XYZ_EULER_ZYZ, 0.849, -0.298+kw_bok, 0.100+kw_bok, -0.004, 1.560, -3.141, 0.074, 0.000, false);
	smoothgen2->load_coordinates(lib::XYZ_EULER_ZYZ, 0.849, -0.298, 	   0.100+kw_bok, -0.004, 1.560, -3.141, 0.074, 0.000, false);
	smoothgen2->load_coordinates(lib::XYZ_EULER_ZYZ, 0.849, -0.298, 	   0.100, 		 -0.004, 1.560, -3.141, 0.074, 0.000, false);
		/*char size[10];
		double size2 = smoothgen2->pose_list_length();
		sprintf(size,"%f",size2);
		sr_ecp_msg->message(size);*/

	smoothgen2->Move();

	//printf("wielkosc listy: %d\n", smoothgen2->pose_list_length());
	//fflush();

	  //smoothgen2->Move();
	  //sr_ecp_msg->message("jest git");
	smoothgen2->reset();

	ecp_termination_notice();
};

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new irp6ot::task::kcz_test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


