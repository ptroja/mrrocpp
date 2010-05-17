// -------------------------------------------------------------------------
// Proces: 	VIRTUAL SENSOR PROCESS (lib::ECP)
// Plik:			vsp_fs.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Metody czujnika sily - po stronie procesu VSP.
// Autor:		tkornuta
// Data:		29.11.2006
// -------------------------------------------------------------------------

#include <time.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"

#include "vsp_time_sensor.h"

namespace mrrocpp {
namespace vsp {
namespace sensor {

time::time(lib::configurator &_config)
	: vsp::sensor::sensor<struct timespec>(_config)
{
}

void time::configure_sensor (void){
   	is_sensor_configured=true;
}

void time::initiate_reading (void){
	// Jesli czujnik nie jest skonfigurowany.
	if(!is_sensor_configured)
	     throw lib::sensor::sensor_error (lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	// Odczyt w porzadku.
	is_reading_ready=true;
}

void time::get_reading (void){
	// Jesli czujnik nie jest skonfigurowany.
	if(!is_sensor_configured)
	     throw lib::sensor::sensor_error (lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);

	// Jezeli nie ma nowego odczytu -> wyslanie starego.
	if(!is_reading_ready)
		return;

	// Odczyt w porzadku.
	from_vsp.vsp_report= lib::VSP_REPLY_OK;

	clock_gettime(CLOCK_REALTIME, &from_vsp.comm_image);

	// Obacny odczyt nie jest "nowy".
	is_reading_ready=false;
} // end: get_reading

VSP_CREATE_SENSOR(time)

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp
