// ------------------------------------------------------------------------
//                                  edp.cc
// 
// EDP_MASTER Effector Driver Master Process
// Driver dla robota IRp-6 na torze - metody: class edp_irp6s_robot
// 
// Ostatnia modyfikacja: styczen 2005
// -------------------------------------------------------------------------

#include <edp/common/edp.h>
#include <edp/common/edp_force_sensor.h>

edp_force_sensor::edp_force_sensor(void) 
{
	gravity_transformation = NULL;
	is_sensor_configured=false;	//!< czujnik niezainicjowany
	is_reading_ready=false;				//!< nie ma zadnego gotowego odczytu	
};

void edp_force_sensor::wait_for_event(void){};			// oczekiwanie na zdarzenie
