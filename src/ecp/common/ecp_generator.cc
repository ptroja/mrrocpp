#include "ecp/common/ecp_generator.h"

ecp_generator::ecp_generator (ecp_task& _ecp_task, bool _is_robot_active)
	:
		ecp_t(_ecp_task)
{
	is_robot_active = _is_robot_active;
	//ecp_t = &(_ecp_task);
	sr_ecp_msg = ecp_t.sr_ecp_msg;
	if (is_robot_active) {
		the_robot = ecp_t.ecp_m_robot;
	} else {
		the_robot = NULL;
	}
	sensor_m.clear();
}

ecp_generator::~ecp_generator()
{}

ecp_generator::ECP_error::ECP_error ( uint64_t err_cl, uint64_t err_no,
                                      uint64_t err0, uint64_t err1 )
		:
		error_class(err_cl),
		error_no(err_no)
{
	error.error0 = err0;
	error.error1 =err1;
}

bool ecp_generator::is_EDP_error (ecp_robot& the_robot) const
{
	// Sprawdzenie czy nie wystapil blad w EDP
	// Funkcja zaklada, ze error_no zostalo zaktualizowane
	// za pomoca conveyor_generator::get_reply
	if ( the_robot.EDP_data.error_no.error0 || the_robot.EDP_data.error_no.error1 ) {
		return true;
	} else {
		return false;
	}
} // end: is_EDP_error()
