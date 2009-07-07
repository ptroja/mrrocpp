#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/irp6_postument/ecp_t_eihcalibration_irp6p.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

//Constructors
eihcalibration::eihcalibration(lib::configurator &_config): common::task::task(_config){
	smoothgen = NULL;
	nrg = NULL;
};

//Desctructor
eihcalibration::~eihcalibration(){

};

//methods for ECP template to redefine in concrete classes
void eihcalibration::task_initialization(void) {

	ecp_m_robot = new ecp_irp6_postument_robot(*this);
	smoothgen = new common::generator::smooth(*this, true);
	nrg = new common::generator::tff_nose_run(*this, 8);
	nrg->configure_pulse_check (true);

/*	//Create cvFraDIA sensor - for testing purposes.
	sensor_m[lib::SENSOR_CVFRADIA] = new ecp_mp::sensor::cvfradia(lib::SENSOR_CVFRADIA,
				"[vsp_cvfradia]", *this,
				sizeof(lib::sensor_image_t::sensor_union_t::deviation_t));

	//Configure sensor.
	sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();
*/
	sr_ecp_msg->message("ECP loaded eihcalibration");
};

void eihcalibration::main_task_algorithm(void ){
	sr_ecp_msg->message("ECP eihcalibration ready");
	//Czekam, az czujnik bedzie skonfigurowany.
	sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
	while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED){
		sensor_m[lib::SENSOR_CVFRADIA]->get_reading();
	}

/*	smoothgen->set_absolute();
	if (smoothgen->load_file_with_path("/mnt/mrroc/trj/box_euler.trj")) {
	  smoothgen->Move();
	}
	smoothgen->reset();
*/
	while(sensor_m[lib::SENSOR_CVFRADIA]->from_vsp.comm_image.sensor_union.chessboard.found == false){
		nrg->Move();
	}

	ecp_termination_notice();
	//ecp_wait_for_stop();
};

}
} // namespace irp6p

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new irp6p::task::eihcalibration(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

